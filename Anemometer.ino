

#include "Anemometer_graphics.h"
#include "limits.h"
#include "float.h"

/******************************************************************************
 * 
 * Global variables and defines
 * 
 ******************************************************************************/

//#define DEBUG_SKIPTITLES  // uncomment to skip titles to speed debugging

/*
  ESP-01 Pinout (from top of board)
                      
               Button SDA
         GND   enable Button1 Optocoupler 
      
               GPIO2  GPIO0 GPIO3
         GND          FLASH  RX
          o      o      o     o  
                               
                               
          o      o      o     o
         TX    CH_PD   RESET VCC
        GPIO1       
        
        SCL    3.3v   3.3v   3.3v   
        Button2
  
 |-----+-------+-------------------+------------------------+-----------------|
 |GPIO | Func. | Boot condition    | Special handling       | Anemometer Func.|
 |-----+-------+-------------------+------------------------+-----------------|
 |GPIO0|FLASH  |                   |Boot fails if pulled LOW|SDA/Button1      |
 |GPIO1|TX,LED |Pulled HIGH at boot|Boot fails if pulled LOW|SCL/Button2      |
 |GPIO2|       |Pulled HIGH at boot|Boot fails if pulled LOW|Button enable    |
 |GPIO3|RX     |Pulled HIGH at boot|                        |Optocoupler      |
 |-----+-------+-------------------+------------------------+-----------------|
*/ 

// GPIO usage
#define IO_SDA    0
#define IO_SCL    1
#define IO_BUTTON 2
#define IO_OPTO   3

// Button variables
#define BUTTON_LONG_MS 1000L
#define BUTTON_LONG_REPEAT_MS 500L

unsigned int pButton1=0;
unsigned long buttonTimer1;

unsigned int pButton2=0;
unsigned long buttonTimer2;
unsigned long buttonLong2=0L;

// control start and stop of collection
unsigned int freezeDisplay=0;

// current, maximum, minimum velocity 
float velocityCurMPH=0.0f;
float velocityMaxMPH=0.0f;
float velocityMinMPH=99999.0f;


// display mode
enum eDisplayMode {
  e_mode_digital,     // One panel
  e_mode_panels,      // Four panels
  e_mode_bar,         // Bar graph
  e_mode_dial,        // Dial graph
  e_mode_inv_digital, // One panel inverted display
  e_mode_inv_panels,  // Four panels inverted display
  e_mode_inv_bar,     // Bar graph inverted display
  e_mode_inv_dial,    // Dial graph inverted display
  e_numDisplayModes
};
unsigned int displayMode=e_mode_digital;

enum eUnits {
  e_mph,
  e_kph,
  e_fps,
  //e_fpm,
  e_mps,
  e_knots,
  e_cfm,
  e_numUnits
};
unsigned int units=e_mph;

// convert mph to other units
float conversions[e_numUnits] ={ 
  1.0f,       // mph to mph,
  1.60934f,   // mph to kph
  1.466666f,  // mph to fps,
  //88.0f,      // mph to fpm,
  0.440704f,  // mph to mps,
  0.868976f,  // mph to knots,
  .001f,      // mph to cfm,
};

// Display texts
char * unitsdisplay[e_numUnits] PROGMEM ={ 
  "mph",
  "kph",
  "ft/s",
  //"ft/m",
  "m/s",
  "Knot",
  "CFM"
};

// declarations
void displayModeDigital();
void displayModePanels();
void displayModeBar();
void displayModeDial();

/******************************************************************************
 * 
 * Conversion from microsecond to miles per hour
 * 
 ******************************************************************************/
// data point structure
struct s_microsecondsToMilesPerHour
{
  unsigned long microseconds;
  float milesPerHour;
};

// Lookup table to convert microseconds to miles per hour
s_microsecondsToMilesPerHour conversionTable[]={
  // ticks, mph, (longes time, slowest speed first)
  {ULONG_MAX,0.0f},
  {1300000L,0.03f},
  {130000L,0.3f},
  {13000L,3.0f},
  {6250L,6.0f},
  {5000L,10.0f},
  {2500L,20.0f},
  {1250L,40.0f},
  {0L,FLT_MAX},
  };

// Convert microseconds to miles per hour via table interpolation
float convertMicrosecondsToMilesPerHour(unsigned long microseconds)
{
  // find index of value with lower time in table (greater speed)
  int unsigned i=1;
  while (microseconds < conversionTable[i].microseconds) ++i;

  // linear extrapolation between values (peculiar order is to maintain positive unsigned long delta calculations)
  float delta=(float) (conversionTable[i-1].microseconds-conversionTable[i].microseconds);
  float percent=((float) (conversionTable[i-1].microseconds)-microseconds)/delta;  //(1 at i, and 0 at i-1)
  float mph=conversionTable[i-1].milesPerHour + (percent * (conversionTable[i].milesPerHour-conversionTable[i-1].milesPerHour));
  return mph;
}

/******************************************************************************
 * 
 * Interrupt Service Routine
 *    Optocoupler pin change
 * 
 ******************************************************************************/
//unsigned long averagemicros;
#define BLADES 7
#define REVOLUTIONS 5
#define SAMPLES (BLADES*REVOLUTIONS)

// Persistent variables modified by the interrupt service routine (ISR)
volatile unsigned long isrlastmicros;
volatile unsigned long isrsum;

volatile unsigned long isrmicros;
//volatile unsigned long microsnow;

// 7 blades, 5 revolutions is 35 events, 32 4.57 revolutions, 128 is 18.29 revolutions
#define RING_BUFFER_POW2 8  
#define RING_BUFFER_SIZE (1<<RING_BUFFER_POW2)
volatile long unsigned ringBuffer[RING_BUFFER_SIZE];
volatile unsigned int ringBufferIndex=0;

// Interrupt for rising edge of optoisolator (once per blade, 7 per revolution)
ICACHE_RAM_ATTR void isrOptocoupler(){
  long unsigned microsnow=micros();
  long unsigned delta=microsnow-isrlastmicros;
  isrlastmicros=microsnow;
  isrmicros = delta;

  // index next element in the ringbuffer
  ringBufferIndex=(ringBufferIndex+1) & (RING_BUFFER_SIZE-1);

  // subtract old ringbuffer delta, and add new delta to rolling sum
  isrsum=isrsum-ringBuffer[ringBufferIndex]+delta;
  
  // store delta time in ring buffer
  ringBuffer[ringBufferIndex]=delta;
  
  // Update index with bounds keeping
}

/******************************************************************************
 * 
 * Setup
 * 
 ******************************************************************************/

// Set display contrast - a very subtle brightness adjustment
void displaycontrast(char unsigned contrast)
{
  display.ssd1306_command(0x81);
  display.ssd1306_command(contrast);
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Initialize I2C
  Wire.begin(IO_SDA, IO_SCL);     // set I2C pins (SDA = GPIO2, SCL = GPIO0), default clock is 100kHz

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC  , 0x3c, false);

  // Display boot sequence logo
  display.clearDisplay();
  displaycontrast(0);
  display.drawBitmap( 0,0, BruttaFaccia, 128,64, 1);
  display.display();
  delay(100);
#ifndef DEBUG_SKIPTITLES
  delay(3000-100);
  display.clearDisplay();
  // SOLID Logo
  display.drawBitmap( 0,0, SOLID_logo, 128,64, 1);
  display.display();
  delay(3000);
  display.clearDisplay();
  display.drawBitmap( 0,0, Workshop88labs, 128,64, 1);
  display.display();
  delay(3000);
#endif    
    
  pinMode(LED_BUILTIN, OUTPUT);   // ESP-01 LED pin is pin 1, same as SCL
  pinMode(IO_OPTO, INPUT_PULLUP);
  pinMode(IO_BUTTON, INPUT_PULLUP);

  // fill ring buffer with 1s delta's (1000000us) 
  for (int unsigned i=0; i<RING_BUFFER_SIZE; ++i) ringBuffer[i]=1000000L; 

  // Initialize optocoupler interrupt service routine
  isrlastmicros=micros();
  attachInterrupt(digitalPinToInterrupt(IO_OPTO), isrOptocoupler, RISING);

  // there is a funny issue at boot where it starts up frozen
  freezeDisplay=1;
}

/******************************************************************************
 * 
 * Main loop
 * 
 ******************************************************************************/\
void loop() {
  unsigned int button1;
  unsigned int button2;

  // Enable button reading on SCL and SDA by making the button output low
  pinMode(IO_BUTTON, OUTPUT);
  digitalWrite(IO_BUTTON, LOW);
  // Read button status on I2C buss lines
  // Note buttons are inverted (0 pressed -> 1 pressed for logic)
  button1= digitalRead(IO_SCL)? 0 : 1;
  button2= digitalRead(IO_SDA)? 0 : 1;
  // Restore weak pullup on button enable pin
  digitalWrite(IO_BUTTON, HIGH);
  pinMode(IO_BUTTON, INPUT_PULLUP);
 
  // Button1
  // detect change
  if (button1^pButton1)
  {  
     // detect change
     if (button1)
     {
        // leading edge
        buttonTimer1=millis();
     } 
     else
     {
        // trailing edge
        buttonTimer1=millis()-buttonTimer1;
        if (buttonTimer1<BUTTON_LONG_MS)
        {
          // short press
          // start stop
          if (velocityMinMPH!=999999.0f)  // check incase there is some interference from the long press - temporary
            freezeDisplay^=1;
        }
     }
  }
  // detect and act on long press
  if (button1)
  {
     if ((millis()-buttonTimer1)>=BUTTON_LONG_MS)
     {
        // long press
        // no long pres sreset, do this every frame until released

        // reset statistics
        // deliberately not setting interrupt related values as they are buffering measurements
        // it is up to the main loop to interpret what is meaningful
        // reset Min and Max (Cur is calculated every loop)
        velocityMinMPH=999999.0f;
        velocityMaxMPH=0.0f;
        freezeDisplay=0;
      }
  }

  // Button2
  // detect change
  if ((button2^pButton2) && (buttonLong2==0))
  {  
     // detect change
     if (button2)
     {
        // leading edge
        // set timer for long press
        buttonTimer2=millis()+BUTTON_LONG_MS;
     } 
     else
     {
        // trailing edge
        // short press
        // cycle units
        if (++units >= e_numUnits)
        {
          units=0;
        }
     }
  }
  // detect and act on long press
  if (button2)
  {
    // if millis > buttonTimer2 with unsigned wrap around
    if ((buttonTimer2-millis()) & 0x80000000L)  
    {
      // long press

      // disable short until release
      buttonLong2=1;

      // reset timer for quick repeat
      buttonTimer2=millis()+BUTTON_LONG_REPEAT_MS;
      
      // cycle display mode
      if (++displayMode >= e_numDisplayModes)
      {
        displayMode=0;
      }
    }
  }
  else
  {
    // disable long button mode enabling short edge presses
    buttonLong2=0;
  }

  // Button processing done, save current state as previous for next loop
  pButton1=button1;
  pButton2=button2;

  if (freezeDisplay==0)
  {
    // calculate velocity to display
    // check micros against isrlastmicros, if too large, velocity is effectively 0
    // start at 1s
    if ((micros()-isrlastmicros)>1000000)
    {
      velocityCurMPH=0.0f;
    }
    else
    {
      // grab index so interrupt doesn't update while in use
      unsigned int index=ringBufferIndex;
      // average the previous number of samples
      unsigned long sum=0L;
      unsigned int i;
      for (i=0; i <(BLADES*REVOLUTIONS); ++i )
      {
        sum+=ringBuffer[(index--) & (RING_BUFFER_SIZE-1)];
        // early exit based on sum if time based average is desired but 3 seconds of high speed wil be a big buffer
      }
      float avemicros=sum/i;
      
      // calculate mph velocity based on average micros time
      velocityCurMPH=convertMicrosecondsToMilesPerHour(avemicros);
    }    
  
    // calculate Min and Max
    if (velocityCurMPH<velocityMinMPH)velocityMinMPH=velocityCurMPH;
    if (velocityCurMPH>velocityMaxMPH)velocityMaxMPH=velocityCurMPH;
  }
  
  // Create display according to display mode
  switch (displayMode & 0x03)
  {
    case e_mode_digital:
      displayModeDigital();
      break;
    case e_mode_panels:
      displayModePanels();
      break;
    case e_mode_bar:
      displayModeBar();
      break;
    case e_mode_dial:
      displayModeDial();
      break;
  }

  // Later half of display modes are inverted 
  if (displayMode & 0x04)
  {
    display.invertDisplay(true);
  }
  else
  {
    display.invertDisplay(false);
  }
  
  // present display
  display.display();
  
  // this is needed but idk why
  delay(1);
}

void displayModeDigital()
{
  // Update display
  display.clearDisplay();    
  display.fillRoundRect(0,0,128,64,8, SSD1306_WHITE);
  
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  
  display.setCursor(16,16);             // Start at bottom left corner
  display.setFont(&FreeSerif9pt7b);
  //display.println(F("Anemometer"));
  
  // Open sans condensed light
  // Roboto condensed
  
  display.setFont(&FreeSansBoldOblique9pt7b);
  display.setCursor(8,16+20*0);
  //display.print(F("Cur"));
  display.println(velocityCurMPH*conversions[units],2);
  display.setFont(&FreeSansOblique9pt7b);
  display.setCursor(8,16+20*1);
  display.print(F("Max: "));display.println(velocityMaxMPH*conversions[units],2);
  display.setCursor(8,16+20*2);
  display.print(F("Min: "));display.println(velocityMinMPH*conversions[units],2);
  
  display.setCursor(128-32,16);
  display.setFont(&FreeSans7pt7b);
  display.print(unitsdisplay[units]);
  
  //display.setCursor(16,34);
  //display.setFont(&FreeSans9pt7b);
  //display.println(F("MAX"));

  // Tiny debug font example we'll keep here
  //display.setFont();
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font
  //display.setTextSize(1);             // Normal 1:1 pixel scale
  //display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //display.setCursor(0,32);             // Start at bottom left corner
  //display.println(stimer1);
  //display.println(stimer2);
}

void displayModePanels()
{
  // Update display
  display.clearDisplay();    
  display.fillRoundRect(0,0,63,31,4, SSD1306_WHITE);
  display.fillRoundRect(64,0,63,31,4, SSD1306_WHITE);
  display.fillRoundRect(0,32,63,31,4, SSD1306_WHITE);
  display.fillRoundRect(64,32,63,31,4, SSD1306_WHITE);

  display.setFont(&FreeSansBoldOblique9pt7b);
  display.setCursor(8,20);
  display.println(velocityCurMPH*conversions[units],2);
  display.setFont(&FreeSansOblique9pt7b);
  display.setCursor(8,64-12);
  display.println(velocityMinMPH*conversions[units],2);
  display.setCursor(64+8,64-12);
  display.println(velocityMaxMPH*conversions[units],2);
  
  display.setCursor(128-32-16,18);
  display.setFont(&FreeSans7pt7b);
  display.print(unitsdisplay[units]);
}


int barx(float fmn, float fmx, float val, int mn, int mx)
{
    float percent = (val-fmn)/(fmx-fmn);
    return mn+(int) (percent * (mx-mn));
}

void displayModeBar()
{
  display.clearDisplay();  
  display.drawRect(0,16,128,32, SSD1306_WHITE);
  // draw scale for units

  float minmph=0.0f;
  float maxmph=50.0f;
  int minx=1;
  int maxx=126;
 
  float xp;
  int x;
  
  for(int speed=0.0f; speed<=maxmph; speed+=10)
  {
    x=barx(minmph,maxmph,speed,minx,maxx);
    display.drawLine(x, 16+2, x, 16+2+12, SSD1306_WHITE);
  }

  for(int speed=0.0f; speed<=maxmph; speed+=5)
  {
    x=barx(minmph,maxmph,speed,minx,maxx);
    display.drawLine(x, 16+2, x, 16+2+8, SSD1306_WHITE);
  }

  for(int speed=0.0f; speed<=maxmph; speed+=1)
  {
    x=barx(minmph,maxmph,speed,minx,maxx);
    display.drawLine(x, 16+2, x, 16+2+3, SSD1306_WHITE);
  }

  x=barx(minmph,maxmph,velocityCurMPH,minx,maxx);
  display.drawLine(x, 16+3, x, 16+32-3, SSD1306_WHITE);

  x=barx(minmph,maxmph,velocityMinMPH,minx,maxx);
  display.drawLine(x, 16+16, x, 16+32-3, SSD1306_WHITE);
  
  x=barx(minmph,maxmph,velocityMaxMPH,minx,maxx);
  display.drawLine(x, 16+16, x, 16+32-3, SSD1306_WHITE);
  
  display.setFont(&FreeSans7pt7b);
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw 'inverse' text
  display.setCursor(8,12);
  display.println(velocityCurMPH*conversions[units],2);
  display.setCursor(8,63);
  display.println(velocityMinMPH*conversions[units],2);
  display.setCursor(64+8,63);
  display.println(velocityMaxMPH*conversions[units],2);
  
  display.setCursor(128-32,12);
  display.print(unitsdisplay[units]);

  
}

float dialLine(float fmn, float fmx, float val, float mn, float mx, int ri, int ro, int cx, int cy)
{
    float percent = (val-fmn)/(fmx-fmn);
    float a = mn + (percent * (mx-mn));
    float s=sin(a);
    float c=cos(a);

    int xo=cx+c*ro;
    int yo=cy-s*ro;
    int xi=cx+c*ri;
    int yi=cy-s*ri;
 
    display.drawLine(xo,yo,xi,yi, SSD1306_WHITE);
}

void displayModeDial()
{
  int cx=64;
  int cy=256;
  int ro=cy-12;
  int ri=ro-32;

  float mina=(90+15)*3.14158/180.0f;
  float maxa=(90-15)*3.14158/180.0f;
  
  float minmph=0.0f;
  float maxmph=50.0f;


  display.clearDisplay();  
  display.drawCircle(cx, cy, ro, SSD1306_WHITE);
  
  /*
  float a;
  for(angle=0.0f; angle<3.14159*2; angle+=3.14159f/18.0f)  
  {
    int x=cx+cos(angle)*ro;
    int y=cy-sin(angle)*ro;
    display.drawLine(cx, cy, (int) x, (int) y, SSD1306_WHITE);
  }
  */

  for(int speed=0.0f; speed<=maxmph; speed+=10)
  {
    dialLine(minmph,maxmph,speed,mina,maxa,ro,ro-2-12,cx,cy);
  }

  for(int speed=0.0f; speed<=maxmph; speed+=5)
  {
    dialLine(minmph,maxmph,speed,mina,maxa,ro-2,ro-2-8,cx,cy);
  }

  for(int speed=0.0f; speed<=maxmph; speed+=1)
  {
    dialLine(minmph,maxmph,speed,mina,maxa,ro-2,ro-2-3,cx,cy);
  }

  dialLine(minmph,maxmph,velocityCurMPH,mina,maxa,ri,ro,cx,cy);
  dialLine(minmph,maxmph,velocityMinMPH,mina,maxa,ri+3,ri+3+8,cx,cy);
  dialLine(minmph,maxmph,velocityMaxMPH,mina,maxa,ri+3,ri+3+8,cx,cy);
  
  display.fillCircle(cx, cy, ri, SSD1306_BLACK);
  display.drawCircle(cx, cy, ri, SSD1306_WHITE);

  display.setFont(&FreeSans7pt7b);
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw 'inverse' text
  display.setCursor(0,12);
  display.println(velocityCurMPH*conversions[units],2);
  display.setCursor(128-32,12);
  display.print(unitsdisplay[units]);
  display.setCursor(8,63);
  display.println(velocityMinMPH*conversions[units],2);
  display.setCursor(64+8,63);
  display.println(velocityMaxMPH*conversions[units],2);  
}

/*
  Anemometer
    3D printed case
      Form factor?
    Electronics
    Power?
      USB rechargable battery (AliExpress)
    Function
      Button
        managed in main loop
        short tap mode
          cfm
          m/s
          f/s
          km/hr
          mpr
        long tap reset
      Optocoupler
        interrupt on pin change record micros()
      display
        max v
        average v
        volume?
      algorithm
        every n clicks? (variable time)
        every s time? (variable clicks)
        calc is ultimately const*clicks/s

  millis()
  micros()
  interrupt on pin change

  128x64 OLED SSD1306 display at I2C address 0x78 (alt. avail. 0x7a) (which is actually 7 bit 0x3C and a 0 start bit)


  
// Direct GPIO access https://github.com/esp8266/Arduino/blob/master/tools/sdk/include/eagle_soc.h
#define ETS_UNCACHED_ADDR(addr) (addr)
#define PERIPHS_GPIO_BASEADDR   0x60000300
#define READ_PERI_REG(addr)     (*((volatile uint32_t *)ETS_UNCACHED_ADDR(addr)))
#define READ_IO(io)              READ_PERI_REG(PERIPHS_GPIO_BASEADDR+((io)*sizeof(uint32_t)))

*/

/* 
 *  Principle of operation:
 *    Microseconds per blade is measured isrmicros
 *    1/isrmicros is blades/microsecond, proportional to speed (distance/time) and flow (volume/time)
 *    a/ismicros allows converstion to speed distance/time (ft/s, mph, m/s, kph)
 *    and incorporating the fixed cross sectional area allows volume/time (cfm)
 * 
 * 
 * 1n914
 */
