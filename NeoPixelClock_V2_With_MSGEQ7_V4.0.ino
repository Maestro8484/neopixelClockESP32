// =====================================================================================================
// Title:    3 RING NEO-PIXEL RGB LED CLOCK V2- with Spectrum Analyser & Simblee control
// Author    STEVE MANLEY
// Function: LED Clock and Audio Spectrum Analyser
// Board:    Arduino Nano 3.1
// Version:  4.0
// Date:     31st December 2015
// License:  The MIT License (See full license at the bottom of this file)
// =====================================================================================================
//   Notes:  Uses 12, 24 & 60 RGB LED Neo-Pixel rings to form a crude clock face,
//           Sparkfun DS3234  Real Time Clock breakout module to provide the time,
//           Sparkfun ADMP401 MEMs microphone breakout module with integral amp
//           MSGEQ7 7 band graphic equaliser chip breakout module.
//           Clock enhanced using 3D printed parts to form clock hands.
//           Displays Time, Date, Various Audio modes. Date and Time can be set.
//           Displays Time in 12 Hour mode, minutes and seconds.
//           Displays Date with year as 2 digit format, month and day of month .
//           ---------------------------------------------------------------------------
//   Clock:  The clock uses full bidirectional SPI to retrieve and set the time & date.
//           It also has a square wave out set at 1Hz, used as a 1 sec interrupt.
//           ---------------------------------------------------------------------------
//     Mic:  The mic breakout is piggybacked directly onto the MSGEQ7 breakout
//           and only requires the 3.3V supply from the Nano.
//           ---------------------------------------------------------------------------
//  MSGEQ7:  The MSGEQ7 breakout has reset and strobe inputs, one analogy input and
//           one output. It uses the 3.3V for the Nano to power it.
//           Audio from the Mic breakout is fed straight into the MSGEQ7 audio input.
//           The 7 frequency bands are presented on the single analog out and the 
//           bands are swapped using the reset and strobe pins.
//           Can be powered from 3.3 or 5V, inputs are 5V tolerant when using 3.3V.
//           ---------------------------------------------------------------------------
// Simblee:  The Simblee is a new micro controller that advertised itself to an App
//           hosted on Apple iOS devices via BlueTooth (Android comming soon). 
//           A Simblee app loaded on the iOS device displays a customisable user
//           interface that can controll or display the Simblee GPIO pins.
//           Simblee GPIO pins 2 & 3 are connected to the Nano I/O pins 5 & 6.
//           Three buttons are represented on the iOS device, mode up and down, the
//           third simulates both buttons pressed simultaniously. The Simblee is more 
//           responsive and reliable than the clocks physical buttons. Requires 3.3V.
//           The GPIO inputs are not 5V tolerant, therefor level shifters are rquired.
//           ---------------------------------------------------------------------------
//   Power:  5V is derived from either a 2A USB or 5V regulated power adaptor.
//           The 5V powers the LED rings and the Arduino Nano. The on-board 3.3V
//           regulator on the Nano powers all other peripherals requiring 3.3V.
//           The 5V input has a 10uF capacitor and the 3.3V supply has a 2200uF
//           capacitor to help reduce significant noise on the MSGEQ7 module.
//           ---------------------------------------------------------------------------
// Analog:   Nano Aref pin tied to 3.3V
// =====================================================================================================
// Revision History:
// -----------------
// V0.0:     Test sequence.
//           ---------------------------------------------------------------------------
// V1.0:     Addition of basic clock with time and date modes.
//           ---------------------------------------------------------------------------
// V2.0:     Addition of date and time setting modes.
//           Addition of audio hardware and audio modes.
//           ---------------------------------------------------------------------------
// V3.0:     Bug fixes to date setting mode.
//           Addition of PacMan mode.
//           ---------------------------------------------------------------------------
// V4.0:     Addition of simblee micro controller to change clock modes from 
//           Apple iOS devices remotely as well as the clock left and right buttons. 
//           The Simblee added more noise to the MSGEQ7 and Mic circuits. Additional 
//           2200uF capacitor added to the 3.3V, plus higher cut off filtering in the 
//           MSGEQ7 read functions.
//           Audio mode 1 code streemlined for efficiency.
// =====================================================================================================



// =====================================================================================================
// includes & definitions
// =====================================================================================================
#include <SPI.h>

#include <Adafruit_NeoPixel.h>

#define  PIXELS              97             // total number of Neo-Pixels
#define  RINGS                4             // led rings output       - pin  4
#define  SIM_UP               7             // mode up   from Simblee - pin 5
#define  SIM_DN               8             // mode down from Simblee - pin 6
#define  BUT_UP               7             // mode up   button       - pin 7
#define  BUT_DN               8             // mode down button       - pin 8
#define  RTC_CS              10             // RTC - SPI chip select  - pin 10
#define  SPI_MOSI            11             // RTC - SPI data out     - pin 11
#define  SPI_MISO            12             // RTC - SPI data in      - pin 12
#define  SPI_SCK             13             // RTC - SPI clock        - pin 13
#define  MSGEQ7_STROBE       16             // MSGEQ7 Strobe          - pin 16
#define  MSGEQ7_RESET        17             // MSGEQ7 Reset           - pin 17
#define  MSGEQ7_AUDIO         1             // MSGEQ7 Audio Out       - pin A1
#define  MSGEQ7_FILTER      200             // cuts off lower values read from MSGEQ7 readings to reduce noise 
#define  ONE_SECOND_INTERUPT  0             // 1Hz on inturrupt 0     - pin  2 
#define  MODE_LIMIT           6             // upper mode limit

Adafruit_NeoPixel rings = Adafruit_NeoPixel(PIXELS, RINGS, NEO_GRB + NEO_KHZ800);

volatile boolean oneSecondElapsed = false;  // flag to acknowlage the 1 second interrupt has occured, must be volatile

byte          ref60[60];                    // reference index to the 60 Neo-Pixel ring pixels
byte          ref24[24];                    // reference index to the 24 Neo-Pixel ring pixels
byte          ref12[12];                    // reference index to the 12 Neo-Pixel ring pixels
long          eqColour[8];                  // crude colour pallet for the one of the audio modes
byte          hour24      = 0;              // hours in 24 hour clock
byte          hour12      = 0;              // hours in 12 hour clock ( am/pm )
byte          minute      = 0;              // minutes
byte          second      = 0;              // seconds
byte          day         = 0;              // day of the month
byte          month       = 0;              // month
byte          year        = 0;              // year in 2 digit format
boolean       hourOffset  = 0;              // flag for offsetting the hour hand when minutes are between 31 - 59
boolean       firstShow   = true;           // bypasses interrupt when clock/date modes are being displayed for the first time after a mode change 
unsigned int  audio[7];                     // contains the 7 channels of audio data from the MSGEQ7 graphic equaliser chip
byte          band;                         // index to which graphic equaliser band is being read
byte          mode        = 0;              // display modes (clock (default), date, audio1, audio1 rotating, audio2, audio2 rotating, PacMan)
byte          modeOld     = mode;           // previous mode
byte          buttonState = 0;              // holds the status of the buttons after being read
byte          counter     = 0;              // temporary index counter
byte          monthLen[]  = {  0,           // month lengths (Feb. is 29 on leap years calculated dynamically)
                              31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };  



// =====================================================================================================
// SETUP
// =====================================================================================================
void setup() {
  Serial.begin(115200);

  resetArrayIndexes();                           // initialises the reference index arrays 
  rings.begin();                                 // initialise the Pixel Rings

  // initialise the SPI & Real Time Clock
  SPI.begin      (                       );      // start the SPI library:
  SPI.setBitOrder( MSBFIRST              );      // set the SPI to send most significant bit first
  SPI.setDataMode( SPI_MODE1             );      // set SPI mode to 1, both mode 1 & 3 should work 
  pinMode        ( RTC_CS,        OUTPUT );      // set SPI chip select pin as digital output
  digitalWrite   ( RTC_CS,           LOW );      // enable data comms to RTC
  SPI.transfer   ( 0x8E                  );      // set RTC control register address
  SPI.transfer   ( 0x60                  );      // SEt TRC control registers SQ wave @1hz, temp compensation, Alarms disabled
  digitalWrite   ( RTC_CS,          HIGH );      // disable data comms to RTC
  
  // initialise the MSGEQ7 & buttons
  pinMode        ( SIM_UP,         INPUT );      // Simblee up   pin set as digital input
  pinMode        ( SIM_DN,         INPUT );      // Simblee down pin set as digital input
  pinMode        ( BUT_UP,         INPUT );      // up   button  pin set as digital input
  pinMode        ( BUT_DN,         INPUT );      // down button  pin set as digital input
  digitalWrite   ( BUT_UP,          HIGH );      // up   button  pin pull up resistor enabled 
  digitalWrite   ( BUT_DN,          HIGH );      // down button  pin pull up resistor enabled 
  pinMode        ( MSGEQ7_RESET,  OUTPUT );      // graphic equaliser reset  pin set as a digital output
  pinMode        ( MSGEQ7_STROBE, OUTPUT );      // graphic equaliser strobe pin set as a digital output
  digitalWrite   ( MSGEQ7_RESET,     LOW );      // graphic equaliser reset  pin initial state
  digitalWrite   ( MSGEQ7_STROBE,   HIGH );      // graphic equaliser strobe pin initial state
  
  // display the time for the first time (time display is the default mode ( mode 0 )
  ReadTimeDate(   );                             // go get the current time
  firstShow  = true;                             // first show bypasses waiting for 1 second interrupt
  displayTime (   );                             // distplay the current time
  
  // start up the 1Hz interrupt from RTC
  attachInterrupt (ONE_SECOND_INTERUPT, registerOneSecond,  FALLING);
  
  
  // initialise crude colour pallet
  eqColour[0] = rings.Color ( 255,   0,   0 );   // red
  eqColour[1] = rings.Color ( 255, 255,   0 );   // yellow
  eqColour[2] = rings.Color (   0, 255,   0 );   // green
  eqColour[3] = rings.Color (   0, 255, 255 );   // cyan
  eqColour[4] = rings.Color (   0,   0, 255 );   // blue
  eqColour[5] = rings.Color (  64,   0, 255 );   // purple
  eqColour[6] = rings.Color ( 255,  64,   0 );   // orange
  eqColour[7] = rings.Color (   0,   0,   1 );   // faint blue
  
}



// =====================================================================================================
// LOOP
// =====================================================================================================
void loop() {

  buttonState = getButtons();
  if ( buttonState == 1 ) { mode++; if ( mode  >   MODE_LIMIT )    mode    =          0; }  // mode up   detected
  if ( buttonState == 2 ) { mode--; if ( mode ==          255 )    mode    = MODE_LIMIT; }  // mode down detected
  if ( mode != modeOld  ) { resetArrayIndexes(); firstShow = true; modeOld = mode;       }  // mode changed do some resets
  if ( buttonState == 3 ) { resetArrayIndexes();                                            // both mode buttons pressed
     if ( mode == 0 )     { setTime();                                                   }  // enter Time setting mode
     if ( mode == 1 )     { setDate();                                                   }  // enter date setting mode
     if ( mode == 2 )     { testSequence();                                              }  // enter led test sequence 
  }
  if ( mode == 0 )        { displayTime();                                               }  // mode 0 - current time ( Default )
  if ( mode == 1 )        { displayDate();                                               }  // mode 1 - current date
  if ( mode == 2 )        { audioMode01();                                               }  // mode 2 - pulsating colours
  if ( mode == 3 )        { audioMode01();                                               }  // mode 3 - pulsating colours plus rotation
  if ( mode == 4 )        { audioMode02();                                               }  // mode 4 - graphic equaliser
  if ( mode == 5 )        { audioMode02();                                               }  // mode 5 - graphic equaliser plus rotation
  if ( mode == 6 )        { pacMan();                                                    }  // display a pacman animation

}



// =====================================================================================================
// DISPLAY CURRENT TIME
// =====================================================================================================
void displayTime() {

  if ( oneSecondElapsed || firstShow ) {         // 1Hz interrupt occured or first show after mode changed to Time mode
    oneSecondElapsed = false;                    // reset interrupt flag 
    firstShow        = false;                    // reset fist show flag
    ReadTimeDate         ( );                    // read date and time
    drawClockFace        ( );                    // draw clock face background
    if (minute > 30) { hourOffset = 1; }         // calc hour hand offset
    else             { hourOffset = 0; }
    
    // set seconds - green, minutes - orange, hours - yellow
    rings.setPixelColor  ( ref60[   second                     ], rings.Color(   0, 128, 0 ) );
    rings.setPixelColor  ( ref60[   minute                     ], rings.Color( 240,  40, 0 ) );
    rings.setPixelColor  ( ref24[ ( hour12 * 2 ) + hourOffset  ], rings.Color( 128, 128, 0 ) );
    rings.setPixelColor  ( ref12[   hour12                     ], rings.Color( 128, 128, 0 ) );
    rings.setPixelColor  ( ref12[ ( hour12 + hourOffset ) % 12 ], rings.Color( 128, 128, 0 ) ); 

    rings.show();                                // display new time
  }

}



// =====================================================================================================
// DISPLAY CURRENT DATE
// =====================================================================================================
void displayDate() {
  
  byte unit;                                    // units  of the 2 year date format
  byte decade;                                  // decade of the 2 year date format

  if ( oneSecondElapsed || firstShow ) {        // 1Hz interrupt occured or first show after mode changed to Time
    oneSecondElapsed = false;                   // reset interrupt flag 
    firstShow        = false;                   // reset fist show flag
    ReadTimeDate  (           );                // read date and time
    unit   =  1 + ( year % 10 );                // calc year unit   pixel number
    decade = 23 - ( year / 10 );                // calc year decade pixel number

    // detect leap year and update February month length 
    if ( ( year % 4 == 0 ) & ( ( year % 100) != 0 ) ) { monthLen[2] = 29; } 
    else                                              { monthLen[2] = 28; }

    drawDateFace();                             // draw date face background 
    
    // set year unit orange, decade Red, month yellow, day green
    rings.setPixelColor( ref24[unit],       rings.Color( 150,  50, 0 ) );
    rings.setPixelColor( ref24[decade],     rings.Color( 200,   0, 0 ) );
    rings.setPixelColor( ref12[month % 12], rings.Color( 128, 128, 0 ) );
    rings.setPixelColor( ref60[day],        rings.Color(   0, 128, 0 ) );

    rings.show();                               // show new date
  }

}



// =====================================================================================================
// DRAW CLOCK FACE BACKGROUND
// =====================================================================================================
void drawClockFace() {
  byte index;
  
  // draw 12 ring faint red, 24 ring faint magenta, 60 ring faint blue with white tick marks every 5
  for ( index =  0; index < 12; index++ ) { rings.setPixelColor( ref12[ index ], rings.Color(  8,  0,  0 ) ); } 
  for ( index =  0; index < 24; index++ ) { rings.setPixelColor( ref24[ index ], rings.Color(  8,  0,  8 ) ); }
  for ( index =  0; index < 60; index++ ) { 
    if( index % 5 == 0)                   { rings.setPixelColor( ref60[ index ], rings.Color( 32, 40, 40 ) ); } 
    else                                  { rings.setPixelColor( ref60[ index ], rings.Color(  0,  0, 16 ) ); } 
  }

}



// =====================================================================================================
// DRAW DATE FACE BACKGROUND
// =====================================================================================================
void drawDateFace() {
  byte index;
  
  // draw part 24 ring faint pink, part 24 ring faint magenta, 12 ring faint red, 
  //      part 60 ring faint blue with white tick marks every 5  
  for ( index =  0; index < 96; index++ ) { rings.setPixelColor(        index,   rings.Color(  0,  0,  0 ) ); }
  for ( index =  0; index < 12; index++ ) { rings.setPixelColor( ref12[ index ], rings.Color(  8,  0,  0 ) ); } 
  for ( index =  1; index < 11; index++ ) { rings.setPixelColor( ref24[ index ], rings.Color(  6,  0, 10 ) ); }
  for ( index = 14; index < 23; index++ ) { rings.setPixelColor( ref24[ index ], rings.Color( 10,  0,  6 ) ); }
  for ( index =  1; index < monthLen[ month ] + 1; index++ ) { 
    if( index % 5 == 0)                   { rings.setPixelColor( ref60[ index ], rings.Color( 32, 40, 40 ) ); } 
    else                                  { rings.setPixelColor( ref60[ index ], rings.Color(  0,  0, 16 ) ); } 
  }

  // draw brighter pink and magenta tick marks on ring24
  rings.setPixelColor( ref24[ 1], rings.Color( 15, 0, 25 ) );
  rings.setPixelColor( ref24[ 6], rings.Color( 15, 0, 25 ) );
  rings.setPixelColor( ref24[18], rings.Color( 25, 0, 15 ) );
  rings.setPixelColor( ref24[23], rings.Color( 25, 0, 15 ) );

}



// =====================================================================================================
// GET BUTTION STATE
// =====================================================================================================
byte getButtons() {

  boolean butRight = 0;                                    // right    button status
  boolean butLeft  = 0;                                    // left     button status
  boolean simUp    = 0;                                    // Simblee mode up status
  boolean simDown  = 0;                                    // Simblee mode dn status
  byte    butState = 0;                                    // combined button status
  
  butRight = !digitalRead ( BUT_UP );                      // read right button state
  butLeft  = !digitalRead ( BUT_DN );                      // read left  button state
  simUp    = !digitalRead ( SIM_UP );                      // read simblee up   state
  simDown  = !digitalRead ( SIM_DN );                      // read simblee down state

  while ( butRight || butLeft || simUp || simDown ) {      // if either button or simblee input is activated loop until non are pressed
    butRight = !digitalRead ( BUT_UP );                    // read right button state
    butLeft  = !digitalRead ( BUT_DN );                    // read left  button state
    simUp    = !digitalRead ( SIM_UP );                    // read simblee up   state
    simDown  = !digitalRead ( SIM_DN );                    // read simblee down state
    if ( butRight || simUp   ) { butState |= 1; }          // if right button has been pressed record it in combined button status 
    if ( butLeft  || simDown ) { butState |= 2; }          // if left  button has been pressed record it in combined button status 
  }
  return ( butState );                                     // return combined button status

}


  
// =====================================================================================================
// BUTTON STATE INCRUMENT/DECROMENT 
//   - accepts an initial value, low & high limit, buttons inc/dec then retrun value within the limits
// =====================================================================================================
byte buttonIncDec( int value, byte lowLimit, byte highLimit ) {

  buttonState = getButtons();
  switch ( buttonState ) {
    case 1:
      value++;
      if ( value > highLimit ) { value = lowLimit; }
      break;
    case 2:
      value--; 
      if ( value < lowLimit ) { value = highLimit; }
      break;
    default:
      break;
  }
  return ( value );

}  

  
  
// =====================================================================================================
// SET THE DATE
//   - when date mode is being displayed, 2 button press will enter date setting mode.
//   - each 2 button press skips between - year, month, day, apply new date. Time is almost unaltered. 
//   - single button press will either inc/dec each value within certain limits.
//   - checks for leap year and that the day stays within the month length.
//   - temp disables interrupts during date setting.
// =====================================================================================================
void setDate(){
  
  byte newYear  = year;                                               // temporarily store current year  as new year  
  byte newMonth = month;                                              // temporarily store current month as new month  
  byte newDay   = day;                                                // temporarily store current day   as new day
  
  noInterrupts();                                                     // temp disable the 1Hz interrupt

  // set year
  // --------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    
    newYear = buttonIncDec( newYear, 0, 99 );                          // inc/dec the new year value making sure it stays within 0 - 99 boundary                        

    if ( ( newYear % 4 == 0 ) & ( ( newYear % 100) != 0 ) ) {          // check if leap year and alter Feb month length accordingly
      monthLen[2] = 29; } else { monthLen[2] = 28; }

    if ( newMonth == 2 && newDay > monthLen[2] ) {                     // in case month is Feb & month length changes, check day of month stays inside month length to prevent date setting errors
      newDay = monthLen[newMonth]; }
 
    day  = newDay;
    year = newYear;
    displayNewDate( 1 );                                               // display new date with year brighter than month and day
  }  


  // set month
  // ---------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    
    newMonth = buttonIncDec( newMonth, 1, 12 );                        // inc/dec the new month making sure it stays within 1 - 12 boundary                    

    if ( newDay > monthLen[newMonth] ) {                               // as month changes check day of month stays inside month length to prevent date setting errors
      newDay = monthLen[newMonth]; }
    
    day   = newDay;
    month = newMonth;
    displayNewDate( 2 );                                               // display new date with month brighter than year and day
  }  


  // set day
  // -------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    
    newDay = buttonIncDec( newDay, 1, monthLen[newMonth] );           // inc/dec the new day making sure it stays within 1 - month length boundary         
    
    day = newDay;
    displayNewDate(  3 );                                             // display new date with day brighter than year and month
  }  


  buttonState = 0;   
  ReadTimeDate();                                                     // read the date and time to capture the current time
  SetTimeDate( newDay, newMonth, newYear, hour24, minute, second );   // apply new date and current time

  interrupts();                                                       // re-enable the interrupts

}
 



// =====================================================================================================
// SET THE TIME
//   - when time mode is being displayed, 2 button press will enter time setting mode.
//   - each 2 button press skips between - hour, minute, second, apply new time. Date is unaltered.
//   - single button press will either inc/dec each value within certain limits.
//   - temp disables interrupts during time setting.
// =====================================================================================================
void setTime(){
  
  byte newHour24 = hour24;                                            // temporarily store current 24 hour as new 24 hour  
  byte newHour12 = hour12;                                            // temporarily store current 12 hour as new 12 hour  
  byte newMinute = minute;                                            // temporarily store current minute  as new minute  
  byte newSecond = second;                                            // temporarily store current second  as new second
  
  noInterrupts();                                                     // temp disable the 1Hz interrupt
  
  //ReadTimeDate();

  // set hour
  // --------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    newHour24 = buttonIncDec( newHour24, 0, 23 );                     // inc/dec the new hour value making sure it stays within 0 - 23 boundary
    newHour12 = newHour24 % 12;                                       // calc 12 hours from 24 hours
    displayNewTime( newHour12, newHour24, newMinute, newSecond, 1 );  // display new time with hours brighter than minutes & seconds
  }

  // set minute
  // ----------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    newMinute = buttonIncDec( newMinute, 0, 59 );                     // inc/dec the new minute value making sure it stays within 0 - 59 boundary
    displayNewTime( newHour12, newHour24, newMinute, newSecond, 2 );  // display new time with minutes brighter than hours & seconds
  }

  // set second
  // ----------
  buttonState = getButtons();
  while ( buttonState != 3 ) { 
    newSecond = buttonIncDec( newSecond, 0, 59 );                     // inc/dec the new second value making sure it stays within 0 - 59 boundary
    displayNewTime( newHour12, newHour24, newMinute, newSecond, 3 );  // display new time with seconds brighter than hours & minutes
  }  

  
  buttonState = 0; 
  ReadTimeDate();  
  SetTimeDate( day, month, year, newHour24, newMinute, newSecond );   // apply current date and new time 
  
  interrupts();                                                       // re-enable the interrupts

}



// =====================================================================================================
// DISPLAY DATE DURING DATE SET MODE
//   - accepts focus, which applies a highlight to either 1 = year, 2 = month, 3 = day 
// =====================================================================================================
void displayNewDate( byte focus ) {
  
  long yrDecBright = rings.Color( 16,  0, 0 );                        // set decade part of the year colour faint red
  long yrUntBright = rings.Color( 16,  4, 0 );                        // set unit   part of the year colour faint orange
  long monthBright = rings.Color( 16, 16, 0 );                        // set month  colour faint yellow
  long dayBright   = rings.Color(  0, 16, 0 );                        // set day    colour faint green
  byte decade;                                                        // holds decade part of the year
  byte unit;                                                          // holds unit   part of the year
  
  unit   =  1 + ( year % 10 );                                        // extract unit   part of the year
  decade = 23 - ( year / 10 );                                        // extracy decade part of the year
  
  drawDateFace();                                                     // draw the date bacground
 
  switch ( focus ) {                                                  // depending on FOCUS value
    case 1:  // year
      yrDecBright  = rings.Color( 200,   0, 0 );                      // set decade part of the year colour bright red
      yrUntBright  = rings.Color( 150,  50, 0 );                      // set unit   part of the year colour bright orange
      break;
    case 2:  // month
      monthBright  = rings.Color( 128, 128, 0 );                      // set month  colour bright yellow
      break;
    case 3:  // day
      dayBright    = rings.Color(   0, 128, 0 );                      // set day    colour bright green
      break;
    default:
      break;
  }

  rings.setPixelColor( ref24[ unit ],       yrUntBright );            // draw Year Units
  rings.setPixelColor( ref24[ decade ],     yrDecBright );            // draw Year Decade
  rings.setPixelColor( ref12[ month % 12 ], monthBright );            // draw Month
  rings.setPixelColor( ref60[ day ],          dayBright );            // draw Day
  
  rings.show();                                                       // display new date

}



// =====================================================================================================
// DISPLAY THE TIME DURING TIME SET MODE
//   - accepts focus, which applies a highlight to either hours, minutes, seconds 
// =====================================================================================================
void displayNewTime( byte hour12, byte hour24, byte minute, byte second, byte focus ) {

  long hourBright   = rings.Color( 16, 16, 0 );                       // set hours   colour faint yellow
  long minuteBright = rings.Color( 30,  5, 0 );                       // set minutes colour faint orange
  long secondBright = rings.Color(  0, 16, 0 );                       // set seconds colour faint green

  drawClockFace();                                                    // draw clock face background

  switch ( focus ) {                                                  // depending on FOCUS value
    case 1:
      hourBright   = rings.Color( 128, 128, 0 );                      // set hours   colour bright yellow
      break;
    case 2:
      minuteBright = rings.Color( 240,  40, 0 );                      // set minutes colour bright orange
      break;
    case 3:
      secondBright = rings.Color(   0, 128, 0 );                      // set seconds colour bright green 
      break;
    default:
      break;
  }
  if ( minute > 30 ) hourOffset = 1; else hourOffset = 0;             // if minutes > 30 offset the hours hand
  rings.setPixelColor ( ref60[ second ],  secondBright );             // draw seconds
  rings.setPixelColor ( ref60[ minute ],  minuteBright );             // draw minutes
  rings.setPixelColor ( ref24[ hour24 ],    hourBright );             // draw 24 hours hour hand
  rings.setPixelColor ( ref12[ hour12 ],    hourBright );             // draw 12 hours hour hand
  
  rings.show();                                                       // display new time
  
}




// =====================================================================================================
// READ MSGEQ7
//   - read 7 analog values from MSGEQ7 graphic equaliser chip and scale for varying LED brightness 
// =====================================================================================================
void readMSGEQ7()
// Function to read 7 band equalizers
{
  unsigned int audioValue;                                             // analog output from MSGEQ7 representing audio amplitude
  
  digitalWrite(MSGEQ7_RESET, HIGH);                                    // toggle MSGEQ7 reset high
  digitalWrite(MSGEQ7_RESET,  LOW);                                    // toggle MSGEQ7 reset low

  for(band = 0; band < 7; band++) {                                    // read each of the MSGEQ7 bands and scale
     digitalWrite(MSGEQ7_STROBE,  LOW);                                // toggle MSGEQ7 strobe low to select next band 
     delayMicroseconds(30);                                            // short delay to allow reading to settle
     audioValue = analogRead(MSGEQ7_AUDIO);                            // read anaog value
     if ( audioValue > MSGEQ7_FILTER ) {                               // filter noise
       audio[band] = map( audioValue, MSGEQ7_FILTER, 1024, 0, 255 ); } // scale analog value to work between 0 - 255
     else { audio[band] = 1; }                                         // if in noise threshold set to 1 for led just lit
     digitalWrite(MSGEQ7_STROBE, HIGH);                                // toggle MSGEQ7 stobe high ready to read next band
  }

}



// =====================================================================================================
// SET DATE AND TIME FUNCTION
//   - function taken directly from the RTC example code on the Sparkfun web site
// =====================================================================================================
int SetTimeDate(int d, int mo, int y, int h, int mi, int s){ 

  int TimeDate [7] = { s, mi, h, 0, d, mo, y };

  for(int i = 0; i <= 6; i++ ) {
    if( i == 3 ) i++;
    int b= TimeDate[i] / 10;
    int a= TimeDate[i] - b * 10;
    if( i == 2 ) { if ( b == 2 ) b = B00000010; else if ( b == 1 ) b = B00000001; }	
    TimeDate[i] = a + ( b << 4 );
	 
    digitalWrite( RTC_CS,  LOW );
    SPI.transfer( i + 0x80     ); 
    SPI.transfer( TimeDate[i]  );
    digitalWrite( RTC_CS, HIGH );
  }

}



// =====================================================================================================
// READ DATE AND TIME
//   - function taken directly from the RTC example code on the Sparkfun web site
// =====================================================================================================
String ReadTimeDate(){

  String temp;
  int TimeDate [7]; //second,minute,hour,null,day,month,year		
  for (int i = 0; i <= 6; i++ ){
    if ( i == 3 ) i++;
    digitalWrite(RTC_CS, LOW);
    SPI.transfer(i+0x00); 
    unsigned int n = SPI.transfer(0x00);        
    digitalWrite(RTC_CS, HIGH);
    int a = n & B00001111;    
    if ( i == 2 ) {	
      int b=(n & B00110000) >> 4; //24 hour mode
      if ( b == B00000010 ) b = 20; else if ( b == B00000001 ) b = 10;
      TimeDate[i] = a + b;
    } 
    else if ( i == 4 ) {
      int b = ( n & B00110000 ) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if ( i == 5 ) {
      int b = ( n & B00010000 ) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if ( i == 6 ) {
      int b=(n & B11110000)>>4;
      TimeDate[i] = a + b * 10;
    }
    else {
      int b = ( n & B01110000 ) >> 4;
      TimeDate[i] = a + b * 10;	
    }
  }
  day    = TimeDate[4];
  month  = TimeDate[5];
  year   = TimeDate[6];
  hour24 = TimeDate[2];
  hour12 = hour24 % 12;
  minute = TimeDate[1]; 
  second = TimeDate[0];

  temp.concat(TimeDate[4]);
  temp.concat("/") ;
  temp.concat(TimeDate[5]);
  temp.concat("/") ;
  temp.concat(TimeDate[6]);
  temp.concat("     ") ;
  temp.concat(TimeDate[2]);
  temp.concat(":") ;
  temp.concat(TimeDate[1]);
  temp.concat(":") ;
  temp.concat(TimeDate[0]);
  
  return(temp);

}



// =====================================================================================================
// TEST SEQUENCE
//   - entered by pressing both buttons in Mode 2, Returns to mode 2 when complete
// =====================================================================================================
void testSequence() {
  rotateHand   ( rings.Color( 255,   0,   0 ), 10 );  // red   single  line   rotation clockwise
  rotateHand   ( rings.Color(   0, 255,   0 ), 10 );  // green single  line   rotation clockwise
  rotateHand   ( rings.Color(   0,   0, 255 ), 10 );  // blue  single  line   rotation clockwise
  backfillHand ( rings.Color( 255,   0,   0 ), 10 );  // red   line  backfill rotating clockwise
  backfillHand ( rings.Color(   0, 255,   0 ), 10 );  // green line  backfill rotating clockwise
  backfillHand ( rings.Color(   0,   0, 255 ), 10 );  // blue  line  backfill rotating clockwise
  colorWipe    ( rings.Color( 255,   0,   0 ), 15 );  // red   pixel backfill rotating clockwise   
  colorWipe    ( rings.Color(   0, 255,   0 ), 15 );  // Green pixel backfill rotating clockwise
  colorWipe    ( rings.Color(   0,   0, 255 ), 15 );  // Blue  pixel backfill rotating clockwise
  rainbow      ( 10 );                                // rainbow effect
  rainbowCycle ( 10 );                                // alternative rainbow effect

}



// =====================================================================================================
// ROTATE A HAND CLOCKWISE
//   - rotates a line (hand) clockwise in the chosed colour
// =====================================================================================================
void rotateHand( long colour, byte wait ) {
  
  for ( int i = 0; i < 120; i++ ) {
    rings.setPixelColor (   i /  2,        colour  );  // set outer  ring pixel
    rings.setPixelColor ( ( i /  5 ) + 60, colour  );  // set middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, colour  );  // set inner  ring pixel
    rings.show();                                      // show the hand
    delay(wait);                                       // short delay
    rings.setPixelColor (   i / 2,         0x000000);  // undraw outer  ring pixel
    rings.setPixelColor ( ( i / 5  ) + 60, 0x000000);  // undraw middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, 0x000000);  // undraw inner  ring pixel
  }

}



// =====================================================================================================
// ROTATE A HAND CLOCKWISE WITH BACKFILL
//   - rotates a line (hand) clockwise in the chosed colour
// =====================================================================================================
void backfillHand( long colour, byte wait ) {
  
  for ( int i = 0; i < 120; i++ ) {
    rings.setPixelColor (   i /  2,        colour  );  // set outer  ring pixel
    rings.setPixelColor ( ( i /  5 ) + 60, colour  );  // set middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, colour  );  // set inner  ring pixel
    rings.show( );                                     // show the hand
    delay( wait );                                     // short delay
  }

}




// =====================================================================================================
// FILL WITH DOTS
//   - fill the display clockwise one pixel at a time in the chosed colour
// =====================================================================================================
void colorWipe( long colour, byte wait ) {
  
  for( int i = 0; i < rings.numPixels(); i++ ) {
      rings.setPixelColor( i, colour );
      rings.show( );
      delay( wait );
  }

}




// =====================================================================================================
// ADAFRUIT RAINBOW
// =====================================================================================================
void rainbow( byte wait) {

  for( int j = 0; j < 256; j++ ) {
    for( int i = 0; i < rings.numPixels(); i++ ) {
      rings.setPixelColor ( i, Wheel ( ( i + j ) & 255 ) );
    }
    rings.show( );
    delay( wait );
  }

}


// =====================================================================================================
// ADAFRUIT RAINBOW CYCLE
//   - this makes the rainbow equally distributed throughout. does it 5 times
// =====================================================================================================
void rainbowCycle( byte wait ) {

  for( int j = 0; j < 256 * 5; j++ ) { // 5 cycles of all colour wheel
    for( int i = 0; i < rings.numPixels(); i++ ) {
      rings.setPixelColor( i, Wheel ( ( ( i * 256 / rings.numPixels() ) + j ) & 255 ) );
    }
    rings.show( );
    delay( wait );
  }

}


// =====================================================================================================
// ADAFRUIT COLOUR WHEEL
//   - Input a value 0 to 255 to get a color value transition is r-g-b & back to r
// =====================================================================================================
uint32_t Wheel( byte WheelPos ) {
  if ( WheelPos < 85 ) {
    return rings.Color( WheelPos * 3, 255 - WheelPos * 3, 0 ); } 
  else if ( WheelPos < 170 ) {
    WheelPos -= 85;
    return rings.Color( 255 - WheelPos * 3, 0, WheelPos * 3 ); }
  else {
   WheelPos -= 170;
   return rings.Color( 0, WheelPos * 3, 255 - WheelPos * 3 ); }
}



// =====================================================================================================
// RESET ARRAY POINTERS TO DEFAULT
// =====================================================================================================
void resetArrayIndexes() {
  
  // reset the ring index arrays, audio mode 3 & 5 changes them during rotation effects
  for (byte index = 0; index < 60; index++) { ref60[index] = index; }
  for (byte index = 0; index < 24; index++) { ref24[index] = index + 60; }
  for (byte index = 0; index < 12; index++) { ref12[index] = index + 84; }

}



// =====================================================================================================
// AUDIO MODE 1
//   - pulsating blocks of colour (mode 2), plus display rotation (mode 3)
// =====================================================================================================
void audioMode01() {
  
  byte temp;
  static boolean rotDir = 0;

  readMSGEQ7();
  // Outer Ring
  // red audio channel 0
  for ( int i = 0; i < 7; i++ ) {
    rings.setPixelColor(ref60[ ( i + 57 ) % 60 ],   rings.Color( audio[0],        0,          0 ) ); }

  //Orange audio channel 1
  for ( int i = 0; i < 6; i++ ) {
    rings.setPixelColor(ref60[ ( i +  4 ) ],        rings.Color( audio[1], audio[1]/3,        0 ) );
    rings.setPixelColor(ref60[ ( i + 51 ) ],        rings.Color( audio[1], audio[1]/3,        0 ) ); }

  // yellow audio channel 2
  for ( int i = 0; i < 5; i++ ) {
    rings.setPixelColor(ref60[ ( i + 10 ) +  0 ],   rings.Color( audio[2], audio[2],          0 ) );
    rings.setPixelColor(ref60[ ( i + 46 ) +  0 ],   rings.Color( audio[2], audio[2],          0 ) ); }

  // blue audio channel 3
  for ( int i = 0; i < 6; i++ ) {
    rings.setPixelColor(ref60[ ( i + 15 ) +  0 ],   rings.Color(        0,        0,   audio[3] ) );
    rings.setPixelColor(ref60[ ( i + 40 ) +  0 ],   rings.Color(        0,        0,   audio[3] ) ); } 

  // violet audio channel 2
  for ( int i = 0; i < 6; i++ ) {
    rings.setPixelColor(ref60[ ( i + 21 ) +  0 ],   rings.Color( audio[4]/3,      0,   audio[4] ) );
    rings.setPixelColor(ref60[ ( i + 34 ) +  0 ],   rings.Color( audio[4]/3,      0,   audio[4] ) ); }

  // crimson audio channel 1
  for ( int i = 0; i < 7; i++ ) {
    rings.setPixelColor(ref60[ ( i + 27 ) +  0 ],   rings.Color( audio[5],        0, audio[5]/3 ) ); }


  // middle ring
  // green audio channel 4
  for ( int i = 0; i < 3; i++ ) {
    rings.setPixelColor(ref24[ ( i + 23 ) % 24 ],   rings.Color( 0,        audio[4],          0 ) );
    rings.setPixelColor(ref24[ ( i +  8 ) +  0 ],   rings.Color( 0,        audio[4],          0 ) );
    rings.setPixelColor(ref24[ ( i + 14 ) +  0 ],   rings.Color( 0,        audio[4],          0 ) ); }

  //cyan audio channel 5
  for ( int i = 0; i < 3; i++ ) {
    rings.setPixelColor(ref24[ ( i +  2 ) +  0 ],   rings.Color( 0,        audio[5],   audio[5] ) );
    rings.setPixelColor(ref24[ ( i + 11 ) +  0 ],   rings.Color( 0,        audio[5],   audio[5] ) );
    rings.setPixelColor(ref24[ ( i + 20 ) +  0 ],   rings.Color( 0,        audio[5],   audio[5] ) ); }

  //various colours audio channel 4 & 5
  for ( int i = 0; i < 3; i++ ) {
    rings.setPixelColor(ref24[ ( i +  5 ) +  0 ],   rings.Color( audio[4],   audio[5],        0 ) );
    rings.setPixelColor(ref24[ ( i + 17 ) +  0 ],   rings.Color( audio[4],   audio[5],        0 ) ); }
 

  // center ring
  // white with yellow, cyan, pink audio channel 3,4,5,6
  for ( int i = 0; i < 3; i++ ) {  
    rings.setPixelColor(ref12[ ( i * 4 ) +  0 ],    rings.Color( audio[0]/2, audio[6],   audio[6] ) );
    rings.setPixelColor(ref12[ ( i * 4 ) +  2 ],    rings.Color( audio[3], audio[4]/2,   audio[6] ) ); }

  for ( int i = 0; i < 6; i++ ) {  
    rings.setPixelColor(ref12[ ( i * 2 ) +  1 ],    rings.Color( audio[6], audio[6],   audio[5]/2 ) ); }

  rings.show();
  delay(2);
  
  // if mode 3 detected rotate the rings, direction of rotation switches at random intervals
  if ( mode == 3) { rotateRings(); }   

}



// =====================================================================================================
// AUDIO MODE 2
//   - 7 band graphic equaliser (mode 5), plus display rotation (mode 6)
// =====================================================================================================
void audioMode02() {

  byte pixel;
  long newColour;
  static int index = 0, loops = 0;
  
  // read the 7 MSGEQ7 channels
  readMSGEQ7();                                                                    // read the 7 audio bands
  
  // rescale the 7 channels
  for ( int i = 0; i < 4;  i++ ) { audio[i] = map ( audio [i], 1, 255, 0, 15 ); }  // re-scale bands 0-3 from 0-255 to 0-15
  for ( int i = 4; i < 6;  i++ ) { audio[i] = map ( audio [i], 1, 255, 0, 12 ); }  // re-scale bands 4-6 from 0-255 to 0-12
  for ( int i = 6; i < 7;  i++ ) { audio[i] = map ( audio [i], 1, 200, 0,  6 ); }  // re-scale bands 4-6 from 0-255 to 0-12
  
  // draw first 4 channels on the outer ring
  for ( int j = 0; j < 4; j++ ) {
    for ( int i = 0; i < 15; i++ ) {
      pixel = ( 15 * j ) + i;
      newColour =  Wheel ( ( ( pixel * 256 / 96 ) + index ) & 255 );
      if ( audio[ j ] >= i )     rings.setPixelColor( ref60[ pixel ], newColour     ); 
      else                       rings.setPixelColor( ref60[ pixel ], eqColour[ 7 ] );
    } 
  }

  // draw next 2 channels on middle ring
  for ( int j = 0; j < 2; j++ ) {
    for ( int i = 0; i < 12; i++ ) {
      pixel = ( 12 * j ) + i;
      newColour =  Wheel ( ( ( (pixel + 60 ) * 256 / 96 ) + index ) & 255 );
      if ( audio[ j + 4 ] >= i ) rings.setPixelColor( ref24[ pixel ], newColour     ); 
      else                       rings.setPixelColor( ref24[ pixel ], eqColour[ 7 ] );
    } 
  }

  // draw last channel on inner ring
  for ( int i = 0; i < 6; i++ ) {
    pixel = i;
    newColour =  Wheel ( ( ( (pixel + 84 ) * 256 / 96 ) + index ) & 255 );
    if ( audio[ 6 ] >= i )       rings.setPixelColor( ref12[ pixel ], newColour     ); 
    else                         rings.setPixelColor( ref12[ pixel ], eqColour[ 7 ] );
    pixel = i + 6;
    newColour =  Wheel ( ( ( (pixel + 84 ) * 256 / 96 ) + index ) & 255 );
    if ( audio[ 6 ] >= i )       rings.setPixelColor( ref12[ pixel ], newColour     ); 
    else                         rings.setPixelColor( ref12[ pixel ], eqColour[ 7 ] );
  }


  rings.show();
  delay(2);
  
  loops = ( loops + 1 ) % 10;
  if ( loops == 9 ) { index = ( index + 1 ) % 256; }
  
  
  // if mode 5 detected rotate the rings, direction of rotation switches at random intervals
  if ( mode == 5) { rotateRings(); }

}



// =====================================================================================================
// PACMAN ANIMATION - displays a munching PacMan
// =====================================================================================================
void pacMan() {
  
  static int   openClose = 1;
  static int      amount = 0;
  static int colourIndex = 0;
  long         newColour = 0;
  byte         lowLimit;
  byte          hiLimit;

  for ( int i = 0; i < 120; i++ ) {
    rings.setPixelColor (   i / 2,         0x000000);  // undraw outer  ring pixel
    rings.setPixelColor ( ( i / 5  ) + 60, 0x000000);  // undraw middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, 0x000000);  // undraw inner  ring pixel
  }

  lowLimit = 30 - amount;
  hiLimit  = 30 + amount;
  newColour =  Wheel ( ( ( ( 1 ) * 256 / 96 ) + colourIndex )  & 255 );

  for ( int i = 0; i < lowLimit; i++ ) {
    rings.setPixelColor (   i /  2,        newColour );  // set outer  ring pixel
    rings.setPixelColor ( ( i /  5 ) + 60, newColour );  // set middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, newColour );  // set inner  ring pixel
  }

  for ( int i = hiLimit; i < 120; i++ ) {
    rings.setPixelColor (   i /  2,        newColour );  // set outer  ring pixel
    rings.setPixelColor ( ( i /  5 ) + 60, newColour );  // set middle ring pixel
    rings.setPixelColor ( ( i / 10 ) + 84, newColour );  // set inner  ring pixel
  }

  colourIndex = ( colourIndex + 1) % 256;
  amount     +=   openClose;
  
  if ( amount < 0  ) { openClose = -openClose; amount =  0; }
  if ( amount > 20 ) { openClose = -openClose; amount = 20; }

  rings.show();
  delay(5);

}




// =====================================================================================================
// ROTATE DISPLAY
//   - rotates the rings in different directions randomly changing directions, each ring seperately
// =====================================================================================================
void rotateRings() {

  static boolean rotDirOut = 0;
  static boolean rotDirMid = 0;
  static boolean rotDirInn = 0;
  byte temp;

  // moderate rotation speed
  counter = (counter + 1) % 60; 

  // randomly change rotation direction
  if ( random(500) ==   1 ) { rotDirOut = !rotDirOut; }
  if ( random(500) == 100 ) { rotDirMid = !rotDirMid; }
  if ( random(500) == 200 ) { rotDirInn = !rotDirInn; }

  // rotate outer ring
  if ( counter % 5 == 0 ) {
    if ( rotDirOut ) {        // clockwise
      temp = ref60[0];
      for (byte index =  0; index < 59; index++) { ref60[index] = ref60[index + 1]; }
      ref60[59] = temp; }
    else {                    // counter clockwise
      temp = ref60[59]; 
      for (byte index = 59; index >  0; index--) { ref60[index] = ref60[index - 1]; }
      ref60[ 0] = temp;
    }
  }
  
  // rotate middle ring
  if ( counter % 10 == 0 ) {
    if ( rotDirMid ) {         //clockwise
      temp = ref24[23];
      for (byte index = 23; index >  0; index--) { ref24[index] = ref24[index - 1]; }
      ref24[ 0] = temp; }
    else {                     // counter clockwise
      temp = ref24[ 0]; 
      for (byte index =  0; index < 23; index++) { ref24[index] = ref24[index + 1]; }
      ref24[23] = temp;
    }
  }
    
  // rotate inner ring
  if ( counter % 15 == 0 ) {
    if ( rotDirInn ) {           // clockwise
      temp = ref12[0];  
      for (byte index =  0; index < 11; index++) { ref12[index] = ref12[index + 1]; }
      ref12[11] = temp; }
    else {                      // counter clockwise
      temp = ref12[11];
      for (byte index = 11; index >  0; index--) { ref12[index] = ref12[index - 1]; }
      ref12[ 0] = temp;
    }
  }
}
  
  
  
// =====================================================================================================
// INTURRUPT ROUTINE
//   - registers 1 second elapsed
// =====================================================================================================
void registerOneSecond() 
{
  oneSecondElapsed = true;
}


