/***
        /$$$$$$              /$$                                  /$$
       /$$__  $$            | $$                                 |__/
      | $$  \ $$ /$$   /$$ /$$$$$$    /$$$$$$  /$$$$$$   /$$$$$$  /$$  /$$$$$$  /$$$$$$$
      | $$  | $$| $$  | $$|_  $$_/   /$$__  $$|____  $$ /$$__  $$| $$ |____  $$| $$__  $$
      | $$  | $$| $$  | $$  | $$    | $$  \__/ /$$$$$$$| $$  \ $$| $$  /$$$$$$$| $$  \ $$
      | $$  | $$| $$  | $$  | $$ /$$| $$      /$$__  $$| $$  | $$| $$ /$$__  $$| $$  | $$
      |  $$$$$$/|  $$$$$$/  |  $$$$/| $$     |  $$$$$$$|  $$$$$$$| $$|  $$$$$$$| $$  | $$
       \______/  \______/    \___/  |__/      \_______/ \____  $$|__/ \_______/|__/  |__/
                                                        /$$  \ $$
                                                       |  $$$$$$/
                                                        \______/
        /$$$$$$  /$$                     /$$
       /$$__  $$| $$                    | $$
      | $$  \__/| $$  /$$$$$$   /$$$$$$$| $$   /$$
      | $$      | $$ /$$__  $$ /$$_____/| $$  /$$/
      | $$      | $$| $$  \ $$| $$      | $$$$$$/
      | $$    $$| $$| $$  | $$| $$      | $$_  $$
      |  $$$$$$/| $$|  $$$$$$/|  $$$$$$$| $$ \  $$
       \______/ |__/ \______/  \_______/|__/  \__/

      Danjovic - June 2022
      https://hackaday.io/project/186065-outragian-clock
*/


//    _ _ _                 _
//   | (_) |__ _ _ __ _ _ _(_)___ ___
//   | | | '_ \ '_/ _` | '_| / -_|_-<
//   |_|_|_.__/_| \__,_|_| |_\___/__/
//

// Date and time functions using a DS3231 RTC connected via I2C and Wire lib

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>

//       _      __ _      _ _   _
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//

#include "Outragian.h"

// button stuff
#define bANALOG        A7
#define th_BUTTON_NONE 768  // voltage thresholds
#define th_BUTTON_SET  256  // 10 bits, 0x3ff = 5V 
#define th_SHORT_PRESS 5    // time thresholds
#define th_LONG_PRESS 65    // in units of 10 ms

// LDR stuff
#define bLDR           A6
#define minLDRres 80
#define maxLDRres 685
#define minBrightness 8
#define maxBrighness 255

//
//    _ __  __ _ __ _ _ ___ ___
//   | '  \/ _` / _| '_/ _ (_-<
//   |_|_|_\__,_\__|_| \___/__/
//

// I2C related
#define sdaHigh()  do { DDR_I2C &= ~(1<<bSDA); PORT_I2C |=  (1<<bSDA); } while (0)
#define sdaLow()   do { DDR_I2C |=  (1<<bSDA); PORT_I2C &= ~(1<<bSDA); } while (0)
#define sdaGet()  ( ( PIN_I2C &   (1<<bSDA) )!=0)
#define sclHigh()  do { DDR_I2C &= ~(1<<bSCL); PORT_I2C |=  (1<<bSCL); } while (0)
#define sclLow()   do { DDR_I2C |=  (1<<bSCL); PORT_I2C &= ~(1<<bSCL); } while (0)
#define I2Cdelay() _delay_us(5)



//               _               _
//    __ _  _ __| |_ ___ _ __   | |_ _  _ _ __  ___ ___
//   / _| || (_-<  _/ _ \ '  \  |  _| || | '_ \/ -_|_-<
//   \__|\_,_/__/\__\___/_|_|_|  \__|\_, | .__/\___/__/
//                                   |__/|_|


typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;
  unsigned       : 1;
} t_seconds;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;
  unsigned       : 1;
} t_minutes;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 2 ;
  unsigned op12_24 : 1 ;
  unsigned         : 1 ;
} t_hours;

typedef struct {
  unsigned dow     : 3 ;  // day of week
  unsigned         : 5 ;
} t_wkDays;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 2;
  unsigned       : 2;
} t_dates;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 1 ;
  unsigned         : 2 ;
  unsigned century : 1 ;
} t_monthsCty;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 4;
} t_years;


typedef struct {
  t_seconds    seconds ;
  t_seconds    minutes ;
  t_hours        hours ;
  t_wkDays       wkDay ;
  t_dates         date ;
  t_monthsCty monthCty ;
  t_years         year ;
} t_timeAndDate;

typedef union  {
  uint8_t rawdata[7];
  t_timeAndDate datetime;
} t_ds3231records;


typedef enum   {
  ST_SHOW_TIME,
  ST_SET_HOUR,
  ST_SET_MINUTE,
  ST_SET_DAY,
  ST_SET_MONTH,
  ST_SET_YEAR
} t_operatingStates;

typedef enum   {
  BT_NONE,
  BT_SET,
  BT_MODE
} t_buttonStates;


typedef enum   {
  EV_NOCHANGE,
  EV_MODE_PULSE,
  EV_MODE_LONG,
  EV_SET_PULSE,
  EV_SET_LONG,
  EV_RELEASE
} t_buttonEvents;

//                   _            _
//    __ ___ _ _  __| |_ __ _ _ _| |_ ___
//   / _/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
//   \__\___/_||_/__/\__\__,_|_||_\__/__/
//

const uint16_t PROGMEM hour8[8] = {  _I,  _J, _G2,  _M,  _L,  _K, _G1,  _H };
const uint16_t PROGMEM phase[8] = { _A2, _B , _C , _D1, _D2, _E , _F , _A1 };
//const PROGMEM uint16_t minute8[8]   = { _A2, _B,  _C, _D1, _D2, _E,  _F, _A1 };  // same as phase[]

const uint16_t PROGMEM tensBCD[10]  = { _100_ ,  _10_,  _20_,  _30_,  _40_,  _50_,  _60_,  _70_,  _80_,  _90_ };
const uint16_t PROGMEM unitsBCD[10] = {  _00_ ,  _01_,  _02_,  _03_,  _04_,  _05_,  _06_,  _07_,  _08_,  _09_ };


//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//

t_ds3231records rtc;
volatile bool readyToGo = false;
volatile t_operatingStates operatingState = ST_SHOW_TIME;

volatile uint16_t displ16SegBuffer;
volatile uint16_t sequentialBuffer[3];

uint8_t display16SegmentBrightness = 128;
bool timeChanged = false;
bool decimalPoint = false;

//                 _       _
//    _ __ _ _ ___| |_ ___| |_ _  _ _ __  ___ ___
//   | '_ \ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
//   | .__/_| \___/\__\___/\__|\_, | .__/\___/__/
//   |_|                       |__/|_|

void advanceHour   (uint8_t delta);
void advanceMinute   (uint8_t delta);

void showTime (void);
void showHour (void);
void showMinute (void);

void showTime16Seg (uint8_t hour, uint8_t minute );

void runClockEngine( t_buttonEvents btEvent );
void runDisplayEngine( void );
void runDisplayError( void );

void runBrightnessControl(void);


//            _              ____
//    ___ ___| |_ _  _ _ __ / /\ \  
//   (_-</ -_)  _| || | '_ \ |  | |
//   /__/\___|\__|\_,_| .__/ |  | |
//                    |_|   \_\/_/
void setup() {
  // Initialize Hardware
  initHW();
  operatingState = ST_SHOW_TIME;
  timeChanged = false;
  //Serial.begin(19200);

}




//    _                ____
//   | |___  ___ _ __ / /\ \  
//   | / _ \/ _ \ '_ \ |  | |
//   |_\___/\___/ .__/ |  | |
//              |_|   \_\/_/


void loop() {
  // wait for tick
  while (! (TIFR1 & (1 << OCF1A) )); // waitReady()
  TIFR1 |=  (1 << OCF1A); // clear flag

  if ( readRtc ( &rtc)  ) {
    runClockEngine( buttonEvents () );
    runDisplayEngine();

    if (timeChanged) {    // TODO check for error during write to RTC
      writeRtc ( &rtc );
      timeChanged = false;
    }

  } else runDisplayError();

  // run the brighness control
  runBrightnessControl();

}




//     __              _   _
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//

// Initialize I/O and peripherals
void initHW( void ) {

  // Initialize I/O ports
  DDRB = (
           ( 1 << 0 ) | //  Segment
           ( 1 << 1 ) | //  Segment
           ( 1 << 2 ) | //  Segment
           ( 1 << 3 ) | //  PWM OUT
           ( 1 << 4 ) | //  Segment
           ( 1 << 5 )  //  DP
         );

  PORTB = (
            ( _OFF << 0 ) | //  Segment
            ( _OFF << 1 ) | //  Segment
            ( _OFF << 2 ) | //  Segment
            ( _OFF << 3 ) | //  PWM OUT
            ( _OFF << 4 ) | //  Segment
            ( _OFF << 5 )  //  DP
          );

  DDRC = (
           ( 1 << 0 ) | //  Segment
           ( 1 << 1 ) | //  Segment
           ( 1 << 2 ) | //  Segment
           ( 1 << 3 ) | //  Segment
           ( 0 << 4 ) | //  SDA (I2C)
           ( 0 << 5 ) | //  SCL (I2C)
           ( 0 << 6 ) | //  LDR ADC
           ( 0 << 7 )  //  BUTTONS

         );

  PORTC = (
            ( _OFF << 0 ) | //  Segment
            ( _OFF << 1 ) | //  Segment
            ( _OFF << 2 ) | //  Segment
            ( _OFF << 3 ) | //  Segment
            ( 1 << 4 ) | //  SDA (I2C)
            ( 1 << 5 ) | //  SCL (I2C)
            ( 0 << 6 ) | //  LDR ADC
            ( 0 << 7 )  //  BUTTONS
          );


  DDRD = (
           ( 1 << 0 ) | //  Segment
           ( 1 << 1 ) | //  Segment
           ( 1 << 2 ) | //  Segment
           ( 1 << 3 ) | //  Segment
           ( 1 << 4 ) | //  Segment
           ( 1 << 5 ) | //  Segment
           ( 1 << 6 ) | //  Segment
           ( 1 << 7 )  //  Segment
         );

  PORTD = (
            ( _OFF << 0 ) | //  Segment
            ( _OFF << 1 ) | //  Segment
            ( _OFF << 2 ) | //  Segment
            ( _OFF << 3 ) | //  Segment
            ( _OFF << 4 ) | //  Segment
            ( _OFF << 5 ) | //  Segment
            ( _OFF << 6 ) | //  Segment
            ( _OFF << 7 )  //  Segment
          );




  // Initialize TIMER 1 for 10ms interval (100 Hz)
  TCCR1A = (
             ( 0 << COM1A1 ) |  // normal port operation
             ( 0 << COM1A0 ) |
             ( 0 << COM1B1 ) |
             ( 0 << COM1B0 ) |
             ( 0 << WGM11  ) |  // WGM[210] = 0b100, CTC mode
             ( 0 << WGM10  )
           );

  TCCR1B = (
             ( 0 << ICNC1 ) |
             ( 0 << ICES1 ) |
             ( 0 << WGM13 ) |
             ( 1 << WGM12 ) |  // WGM[210] = 0b100, CTC mode
             ( 0 << CS12  ) |
             ( 1 << CS11  ) |  // CS2[210] = 0b010, presc 8
             ( 0 << CS10 )
           );

  TIMSK1 = (
             (0 << ICIE1  ) |  // no interrupts
             (0 << OCIE1B ) |
             (0 << OCIE1A ) |
             (0 << TOIE1  )
           );

  TCNT1  = 0;        // initialize counter value to 0
  OCR1A = 19999;     // = ( (16000000/8)  / 100 ) - 1
  TIFR1 |= (1 << OCF1A); // clear any pending interrupt flag





  // Initialize TIMER2 for PWM at PB3 (D11)
  TCCR2A = (
             ( 1 << COM2A1 ) |  // COM2A[10] = 0b10, normal pwm
             // COM2A[10] = 0b11, inverted pwm
#if defined CA
             ( 1 << COM2A0 ) |   // normal pwm
#elif defined CC
             ( 0 << COM2A0 ) |   // inverted pwm
#else
#error Must Define CA or CC
#endif
             ( 0 << COM2B1 ) |
             ( 0 << COM2B0 ) |
             ( 1 << WGM21  ) |  // WGM[210] = 0b011, fast PWM top = 0xff
             ( 1 << WGM20  )
           );

  TCCR2B = (
             ( 0 << FOC2A ) |
             ( 0 << FOC2B ) |
             ( 0 << WGM22 ) |  // WGM[210] = 0b011, fast PWM top = 0xff
             ( 0 << CS22  ) |  // CS2[210] = 0b001, no presc, PWM 62500Hz
             ( 0 << CS21  ) |
             ( 1 << CS20  )
           );



  TIMSK2 = (
             (0 << OCIE2B ) |  // no interrupts
             (0 << OCIE2A ) |
             (0 << TOIE2  )
           );

  OCR2A = 64; // quarter brightness


}
//



// ******************************************************************************************************************
//
// Button handling
//
// Read Button press
t_buttonStates getButton (void) {
  uint16_t sample = analogRead(bANALOG ) ;

  if (sample > th_BUTTON_NONE ) return BT_NONE;       //  > 768
  else if (sample > th_BUTTON_SET) return BT_SET;     // > 256
  else return BT_MODE;
}
//

// Get button events - pulse, long press
t_buttonEvents buttonEvents (void) {
  static uint8_t btModeCount = 0, btSetCount = 0;
  uint8_t btEvent = EV_NOCHANGE;

  uint8_t buttonNow = getButton();

  switch (buttonNow ) {

    case BT_SET:
      if ( btSetCount < th_LONG_PRESS ) {
        btSetCount++;                         // will stop count at threshold
        if ( btSetCount >= th_LONG_PRESS ) { // and generate an unique long press event
          btEvent  = EV_SET_LONG;
        }
      }
      break;

    case BT_MODE:
      if ( btModeCount < th_LONG_PRESS ) {
        btModeCount++ ;                       //  will stop count at threshold
        if ( btModeCount >= th_LONG_PRESS ) {  // and generate an unique long press event
          btEvent  = EV_MODE_LONG;
        }
      }
      break;

    case BT_NONE:
      if ( btSetCount >= th_SHORT_PRESS && btSetCount < th_LONG_PRESS ) {
        btEvent = EV_SET_PULSE;
      }

      if ( btModeCount >= th_SHORT_PRESS && btModeCount < th_LONG_PRESS ) {
        btEvent = EV_MODE_PULSE;
      }

      btSetCount = 0;
      btModeCount = 0;
      break;

    default:
      btEvent = EV_NOCHANGE;
      break;
  }
  return btEvent;
}
//



// ******************************************************************************************************************
//
// 16 segment
//
// Shift bits out to 16 segment display
void shiftOut16seg ( uint16_t dbuffer, bool dPoint ) {

  if (dbuffer & _A1 ) SET_A1   ; else  CLEAR_A1  ;
  if (dbuffer & _A2 ) SET_A2   ; else  CLEAR_A2  ;
  if (dbuffer & _B  ) SET_B    ; else  CLEAR_B   ;
  if (dbuffer & _C  ) SET_C    ; else  CLEAR_C   ;
  if (dbuffer & _D1 ) SET_D1   ; else  CLEAR_D1  ;
  if (dbuffer & _D2 ) SET_D2   ; else  CLEAR_D2  ;
  if (dbuffer & _E  ) SET_E    ; else  CLEAR_E   ;
  if (dbuffer & _F  ) SET_F    ; else  CLEAR_F   ;
  if (dbuffer & _G1 ) SET_G1   ; else  CLEAR_G1  ;
  if (dbuffer & _G2 ) SET_G2   ; else  CLEAR_G2  ;
  if (dbuffer & _H  ) SET_H    ; else  CLEAR_H   ;
  if (dbuffer & _I  ) SET_I    ; else  CLEAR_I   ;
  if (dbuffer & _J  ) SET_J    ; else  CLEAR_J   ;
  if (dbuffer & _K  ) SET_K    ; else  CLEAR_K   ;
  if (dbuffer & _L  ) SET_L    ; else  CLEAR_L   ;
  if (dbuffer & _M  ) SET_M    ; else  CLEAR_M   ;

  if (dPoint        ) SET_DP   ; else  CLEAR_DP  ;

}



// ******************************************************************************************************************
//
// Date and Time calculation and handling stuff
//

// advance current hour
void advanceHour   (uint8_t delta) {
  uint8_t d = ((delta
                + 10 * (uint8_t)rtc.datetime.hours.tens
                + (uint8_t)rtc.datetime.hours.units )   ) % 24;

  rtc.datetime.hours.tens = ( (uint8_t) d / 10 )  & 0b00000011;
  rtc.datetime.hours.units = ( (uint8_t) d % 10 ) & 0b00001111;

  timeChanged = true; // update rtc data
}
//

// advance current minute
void advanceMinute   (uint8_t delta) {
  uint8_t d = (delta
               + 10 * (uint8_t)rtc.datetime.minutes.tens
               + (uint8_t)rtc.datetime.minutes.units ) % 60;

  rtc.datetime.minutes.tens = ( (uint8_t) d / 10 )  & 0b00000111;
  rtc.datetime.minutes.units = ( (uint8_t) d % 10 ) & 0b00001111;

  timeChanged = true; // update rtc data
}
//





// ******************************************************************************************************************
//
// Display contents handling
//

// set 3 digits according to the curent hour
void showHour (void ) {
  sequentialBuffer[0] = _HOUR_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.hours.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.hours.units);
}
//

// set 3 digits according to the curent minute
void showMinute (void ) {
  sequentialBuffer[0] = _MINUTE_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.minutes.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.minutes.units);
}
//

// ******************************************************************************************************************
//
//  Clock engine stuff
//
// clock engine
void runClockEngine( t_buttonEvents btEvent ) {

  switch ( operatingState ) {
    case ST_SHOW_TIME:
      //    if ( btEvent == EV_MODE_LONG ) advanceDisplayMode();
      if ( btEvent == EV_SET_LONG ) operatingState = ST_SET_HOUR;
      showTime();
      break;

    case ST_SET_HOUR:
      showHour();
      if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;
      if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_MINUTE;
      if ( btEvent == EV_MODE_PULSE ) advanceHour(1);
      if ( btEvent == EV_MODE_LONG )  advanceHour(10);
      break;

    case ST_SET_MINUTE:
      showMinute();
      if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;
      if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_HOUR;
      if ( btEvent == EV_MODE_PULSE ) advanceMinute(1);
      if ( btEvent == EV_MODE_LONG )  advanceMinute(10);
      break;

  }
  //
}


// ******************************************************************************************************************
//
//  Display engine stuff
//


// Display engine
void runDisplayEngine( void ) {
  static uint8_t sequentialCounter = 0;
  static uint16_t d16buf = 0;

  if ( operatingState == ST_SHOW_TIME ) { // Display mode
    d16buf = displ16SegBuffer;
    sequentialCounter = 0;

  } else {  // Setup mode, show buffer sequentially



    switch (sequentialCounter) {
      case 0:
      case 1:
        // case 1: // show first element
        d16buf = sequentialBuffer[0];
        break;

      case 44:  // show second element
        d16buf = sequentialBuffer[1];
        break;

      case 88: // show third element
        d16buf = sequentialBuffer[2];
        break;

      case 40: // blank
      case 84: // blank
      case 128: // blank
        d16buf = 0;
        break;

    }

    sequentialCounter++;
    if (sequentialCounter > 192 ) sequentialCounter = 0;

  }

  // update displays

  // shift out data to 16 segment
  shiftOut16seg (d16buf, decimalPoint);



}
//

// Show time on 16 segment display (ultradian cycle)
void showTime16Seg (uint8_t hour, uint8_t minute ) {
  uint8_t k;

  uint8_t inner = ( (hour * 60 + minute) / 90 ) % 8;
  uint8_t outer = ( (hour * 60 + minute) / 10 ) % 9;

  displ16SegBuffer  = pgm_read_word( &(hour8[inner]) );// 90 minute interval

  while ( outer > 0) { // add 10 minute intervals passed
    k = (outer - 1 + inner) % 8;
    displ16SegBuffer |= pgm_read_word( &(phase[k]) );
    outer--;
  }

  // AM/PM point
  if ( hour >= 12) decimalPoint = true; else decimalPoint = false;

}
//



// ******************************************************************************************************************
//
// RTC chip handling
//


// write time data on clock Chip using I2C
bool writeRtc ( t_ds3231records *t) {
  uint8_t i;
  I2Cstart();
  if (!I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {
    I2Cwrite( 0x00 ); // register address, 1st clock register
    for ( i = 0 ; i < 7 ; i++)
      I2Cwrite(t->rawdata[i]);
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//

// read time data from clock Chip using I2C
bool readRtc ( t_ds3231records *t) {
  uint8_t i;
  I2Cstart();
  if ( !I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {
    I2Cwrite( 0x00); // register address, 1st clock register
    I2Cstart();  // repeated start
    I2Cwrite((uint8_t)(RTC_ADDRESS << 1)  | 1);
    for ( i = 0 ; i < 6 ; i++) {
      t->rawdata[i] = I2Cread ( sendACK );
    }
    t->rawdata[i] = I2Cread ( sendNACK );       // NACK on last bit
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//




// ******************************************************************************************************************
//
// Soft I2C
//
void I2Cstart() {
  sdaHigh(); I2Cdelay();
  sclHigh(); I2Cdelay(); // sda = 1;  scl = 1;
  sdaLow();  I2Cdelay(); // sda = 0;
  sclLow();
}
//

void I2Cstop() {
  sdaLow();  I2Cdelay(); // sda = 0;  sda = 0;
  sclHigh(); I2Cdelay(); // scl = 1;  scl = 1;
  sdaHigh();             // sda = 1;
}
//

bool I2Cwrite(uint8_t d) {
  uint8_t i;
  bool nack;
  for (i = 0; i < 8; i++) {
    if (d & 0x80)   // write data bit, msb first
      sdaHigh();
    else sdaLow();
    I2Cdelay(); // give time to settle data
    sclHigh(); I2Cdelay();  sclLow(); // pulse clock
    d = d << 1; // next bit
  }
  // now get the ack
  sdaHigh(); I2Cdelay();  // release data line
  sclHigh(); I2Cdelay();  // release clock line
  nack = sdaGet();  // get nack bit
  sclLow();// clock low
  return nack;
}
//

uint8_t I2Cread(bool nack) {
  uint8_t i, d;

  d = 0;
  sdaHigh();             // release data line and
  sclLow(); I2Cdelay();  // pull down clock line and wait to write a bit
  for (i = 0; i < 8; i++)  {
    sclHigh(); I2Cdelay(); // release clock line to read the data
    d = d << 1;
    if (sdaGet() ) d |= 1; // read data bit, msb first
    sclLow(); I2Cdelay();  // pull clock down to allow next bit
  }
  // give ACK / NACK
  if ( nack ) sdaLow(); else sdaHigh();

  sclHigh(); I2Cdelay(); // Pulse clock
  sclLow(); I2Cdelay();  //

  sdaHigh(); // release the data line
  return d;
}
//



//
//   Display error
//
// RTC fail display routine
void runDisplayError( void ) {
  static uint8_t count = 0;
  static uint8_t j = 0;

  // blink star pattern
  if (count == 0 ) {
    shiftOut16seg ( _STAR_ , false );

  } else if (count == 60 )
    shiftOut16seg ( _BLANK_ , false);

  if (count < 100 ) count++; else count = 0;

}
//


void showTime       (void) {
  uint8_t hour   = 10 * (uint8_t)rtc.datetime.hours.tens    + (uint8_t)rtc.datetime.hours.units ;
  uint8_t minute = 10 * (uint8_t)rtc.datetime.minutes.tens  + (uint8_t)rtc.datetime.minutes.units ;

  showTime16Seg (hour, minute );

}
//



void runBrightnessControl(void) {

  uint16_t ldrResult = analogRead( bLDR ) ;
  if (ldrResult < minLDRres ) ldrResult = minLDRres;
  if (ldrResult > maxLDRres) ldrResult = maxLDRres;
  OCR2A = uint8_t(map(ldrResult, minLDRres, maxLDRres, minBrightness, maxBrighness));

}
//



//     __ _      _
//    / _(_)_ _ (_)___
//   |  _| | ' \| (_-<
//   |_| |_|_||_|_/__/
//
