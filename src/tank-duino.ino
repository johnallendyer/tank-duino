///////////////////////////////////////////////////////////////////
// Current Satellite LED+ Controller  V4.1 - MODIFIED FOR KMAN v2//
//   Indychus...Dahammer...mistergreen @ plantedtank.net         //
//   This code is public domain.  Pass it on.                    //
//   Confirmed on Arduino UNO 1.0.5                              //
//   Req. Time, TimeAlarms, RTClib, IRremote                     //
///////////////////////////////////////////////////////////////////
//
// This version uses Ken Shirriff's IRremote library to Rx/Tx the IR codes
// http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
//
// This code does NOT use PIN 13 on the Uno, as do previous versions
// Instead PIN 3, which is a PWM pin, is used. So you'll need to connect
// your LED to PIN 3 instead of PIN 13 for it to work.  FOR MEGA USE PIN 9
//
// You can test the IR commands via the Arduino software's serial monitor
// by sending in a value from 1 - 32. Values follow the remote control,
// left to right, top to bottom (e.g 1 = Orange, 2 = Blue, 21 = Moon1, etc)
//
// Install LCD per instructions at http://learn.adafruit.com/character-lcds/overview

// TODO:
//   - relay control for co2
//   - interior cabinet light

#include <Wire.h>
#include <RTClib.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>
#include <LiquidCrystal.h>
#include <LCD.h>                //added for use with I2C LCD backpack
#include <LiquidCrystal_I2C.h>  //added for use with I2C LCD backpack
#include <OneWire.h>
#include <DallasTemperature.h>

RTC_DS1307 RTC;
IRsend irsend;

static FILE uartout = {0};   // UART FILE structure
static FILE lcdout = {0} ;   // LCD FILE structure

//---------- LCD SETUP
// #define LCD_COLS 16      // Number of columns on the LCD (e.g. 16, 20, etc)
// #define LCD_ROWS 2       // Number of rows on the LCD (e.g. 2, 4, etc)
#define LCD_COLS 20      // Number of columns on the LCD (e.g. 16, 20, etc)
#define LCD_ROWS 4       // Number of rows on the LCD (e.g. 2, 4, etc)

//BEGIN new section added for I2C LCD backpack
#define I2C_ADDR 0x27 // 0x3f = I2C address for LCD backpack: YwRobot Arduino LCM1602 IIC V1
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin, BACKLIGHT_PIN, POSITIVE);
//END new section added for I2C LCD backpack

//---------- THERMOMETER SETUP - pin 2
OneWire oneWire(2);
DallasTemperature sensor(&oneWire);
int dailyLow = 0;
int dailyHigh = 0;

// Define pins for CO2 relays
#define CO2_RELAY1 4
#define CO2_RELAY2 5

// Current Satellite+ IR Codes (NEC Protocol)
unsigned long codeHeader = 0x20DF; // Always the same
// Remote buttons listed left to right, top to bottom
const unsigned int arrCodes[32] PROGMEM = {
    0x3AC5,  // 1 -  Orange
    0xBA45,  // 2 -  Blue
    0x827D,  // 3 -  Rose
    0x02FD,  // 4 -  Power On/Off
    0x1AE5,  // 5 -  White
    0x9A65,  // 6 -  FullSpec
    0xA25D,  // 7 -  Purple
    0x22DD,  // 8 -  Play/Pause
    0x2AD5,  // 9 -  Red Up
    0xAA55,  // 10 - Green Up
    0x926D,  // 11 - Blue Up
    0x12ED,  // 12 - White Up
    0x0AF5,  // 13 - Red Down
    0x8A75,  // 14 - Green Down
    0xB24D,  // 15 - Blue Down
    0x32CD,  // 16 - White Down
    0x38C7,  // 17 - M1 Custom
    0xB847,  // 18 - M2 Custom
    0x7887,  // 19 - M3 Custom
    0xF807,  // 20 - M4 Custom
    0x18E7,  // 21 - Moon 1
    0x9867,  // 22 - Moon 2
    0x58A7,  // 23 - Moon 3
    0xD827,  // 24 - Dawn/Dusk
    0x28D7,  // 25 - Cloud 1
    0xA857,  // 26 - Cloud 2
    0x6897,  // 27 - Cloud 3
    0xE817,  // 28 - Cloud 4
    0x08F7,  // 29 - Storm 1
    0x8877,  // 30 - Storm 2
    0x48B7,  // 31 - Storm 3
    0xC837   //32 - Storm 4
};

// These are the messages that print on the serial monitor & lcd when each IR code is sent
#define MAX_MSG_LEN 13  // Maximum length of the arrMSG messages
const char arrMSG[][MAX_MSG_LEN + 1] PROGMEM = {
    "Orange",     "Blue",         "Rose",       "On/Off",
    "White",      "Full Spec",    "Purple",     "Play/Pause",
    "Red Up",     "Green Up",     "Blue Up",    "White Up",
    "Red Down",   "Green Down",   "Blue Down",  "White Down",
    "Custom 1",   "Custom 2",     "Custom 3",   "Custom 4",
    "Moon 1",     "Moon 2",       "Moon 3",     "Dawn/Dusk",
    "Cloud 1",    "Cloud 2",      "Cloud 3",    "Cloud 4",
    "Storm 1",    "Storm 2",      "Storm 3",    "Storm 4"
};

void SetAlarms() {
    // Set up alarms here - make sure dtNBR_ALARMS in TimeAlarms.h is
    // at least equal to the number of alarms declared below

    // Reset daily high/low temperature
    Alarm.alarmRepeat( 0, 00, 0, ResetDailyTemps);  // 12AM

    // Lights
    Alarm.alarmRepeat( 6, 00, 0, PowerOnOff);   // 6AM
    Alarm.alarmRepeat( 7, 00, 0, DawnDusk);     // 7AM

    // CO2 - on at 7AM
    Alarm.alarmRepeat( 7, 00, 0, TurnOnCO2);    // 7AM

    // Lights
    Alarm.alarmRepeat(10, 00, 0, Cloud2);       // 10AM
    Alarm.alarmRepeat(11, 00, 0, FullSpec);     // 11AM
    Alarm.alarmRepeat(16, 00, 0, Cloud2);       // 4PM

    // CO2 - off at 6PM
    Alarm.alarmRepeat(18, 00, 0, TurnOffCO2);    // 6PM


    // Lights
    Alarm.alarmRepeat(18, 00, 0, DawnDusk);     // 6PM
    Alarm.alarmRepeat(20, 00, 0, Moon2);        // 8PM
    Alarm.alarmRepeat(22, 00, 0, PowerOnOff);   // 10AM

    // Timer for temperature sensor
    Alarm.timerRepeat(5, PrintTemp);  // Update temp every 5 seconds
}

void setup() {
    Wire.begin();
    RTC.begin();
    Serial.begin(9600);
    lcd.begin(LCD_COLS, LCD_ROWS);
    sensor.begin();   // Start temp sensor
    pinMode(CO2_RELAY1, OUTPUT);
    pinMode(CO2_RELAY2, OUTPUT);

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout;  // Output stdout to UART

    // fill in lcd file descriptor (we'll use fprintf to output to it)
    fdev_setup_stream (&lcdout, lcd_putchar, NULL, _FDEV_SETUP_WRITE);

    if (!RTC.isrunning()) {
        // If no RTC is installed, set time to compile time at each reset
        printf_P(PSTR("RTC is NOT running!\n"));  // Store this string in PROGMEM
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }

    setSyncProvider(syncProvider);     // reference our syncProvider function instead of RTC_DS1307::get()

    printf_P(PSTR("Time: %02d:%02d:%02d\n"), hour(), minute(), second());  // Print the time
    SetAlarms();  // Set up above alarms

    // Print available SRAM for debugging, comment out if you want
    printf_P(PSTR("SRAM: %d\n"), freeRam());

    printf_P(PSTR("To test IR codes, send 1 - 32\n"));
}

void loop() {
    if (Serial.available() > 0) {
        delay(5); //Wait for transmission to finish
        TestCodes(SerialReadInt());  // Go grab IR code and send it
    }
    Alarm.delay(100);   // Service alarms & wait (msec)

    char ampm[] = "AM";
    if (isPM()) ampm[0] = 'P';

    lcd.setCursor(0,0);
    fprintf(&lcdout, "%02d:%02d %s", hourFormat12(), minute(), ampm);  // Print the time HH:MM to the lcd
}

time_t syncProvider() {
  //this does the same thing as RTC_DS1307::get()
  return RTC.now().unixtime();
}

int SerialReadInt() {
    // Reads first 2 bytes from serial monitor; anything more is tossed
    byte i;
    char inBytes[3];
    char * inBytesPtr = &inBytes[0];  // Pointer to first element

    for (i = 0; i < 2; i++)           // Only want first 2 bytes
        inBytes[i] = Serial.read();
    inBytes[i] =  '\0';               // Put NULL character at the end
    while (Serial.read() >= 0)        // If anything else is there, throw it away
        ; // do nothing
    return atoi(inBytesPtr);          // Convert to decimal
}

void TestCodes(int cmd) {
    // Handles commands sent in from the serial monitor
    if (cmd >= 1 && cmd <= 32) {
        // cmd must be 1 - 32
        SendCode(cmd - 1, 1);
    } else {
        printf_P(PSTR("Invalid Choice\n"));
    }
}

void SendCode(int cmd, byte numTimes) {
    // cmd = the element of the arrCode[] array that holds the IR code to be sent
    // numTimes = number of times to emmit the command
    // Shift header 16 bits to left, fetch code from PROGMEM & add it with bitwise or
    unsigned long irCode = (codeHeader << 16) | pgm_read_word_near(arrCodes + cmd);
    for ( byte i = 0; i < numTimes; i++) {
        irsend.sendNEC(irCode, 32); // Send/emmit code
        delay(100);
    }
    // Print the string associated with the IR code & the time
    printf("%S: %02d:%02d:%02d\n", arrMSG[cmd], hour(), minute(), second());

    lcd.setCursor(0,1);
    lcd.print("LED Mode: ");
    fprintf(&lcdout, "%-10S", arrMSG[cmd]);
}

void ResetDailyTemps() {
    dailyLow = 0;
    dailyHigh = 0;
}

void PrintTemp() {
    sensor.requestTemperatures();
    int temp = (int)(sensor.getTempCByIndex(0) * 1.8 + 32.5);
    char deg = (char)223;

    if (temp < dailyLow || dailyLow == 0) {
        dailyLow = temp;
    }

    if (temp > dailyHigh) {
        dailyHigh = temp;
    }

    lcd.setCursor(0,2);
    fprintf(&lcdout, "Temp: %d%c (%d%c/%d%c)", temp, deg, dailyLow, deg, dailyHigh, deg);
}

void TurnOnCO2() {
    digitalWrite(CO2_RELAY1, 0);
    digitalWrite(CO2_RELAY2, 1);
    lcd.setCursor(0,3);
    lcd.print("CO2: On");
}

void TurnOffCO2() {
    digitalWrite(CO2_RELAY1, 1);
    digitalWrite(CO2_RELAY2, 0);
    lcd.setCursor(0,3);
    lcd.print("CO2: Off");
}

int freeRam() {
    // Returns available SRAM
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Output function for stdout redirect
static int uart_putchar(char c, FILE *stream) {
    // Serial write fumction
    Serial.write(c);
    return 0;
}

// Output function for lcd output
static int lcd_putchar(char ch, FILE* stream) {
    // lcd write function
    lcd.write(ch);
    return (0);
}

// IR Code functions, called by alarms
void Orange()     { SendCode(0, 2);  }
void Blue()       { SendCode(1, 2);  }
void Rose()       { SendCode(2, 2);  }
void PowerOnOff() { SendCode(3, 1);  }
void White()      { SendCode(4, 2);  }
void FullSpec()   { SendCode(5, 2);  }
void Purple()     { SendCode(6, 2);  }
void Play()       { SendCode(7, 1);  }
void RedUp()      { SendCode(8, 1);  }
void GreenUp()    { SendCode(9, 1);  }
void BlueUp()     { SendCode(10, 1); }
void WhiteUp()    { SendCode(11, 1); }
void RedDown()    { SendCode(12, 1); }
void GreenDown()  { SendCode(13, 1); }
void BlueDown()   { SendCode(14, 1); }
void WhiteDown()  { SendCode(15, 1); }
void M1Custom()   { SendCode(16, 2); }
void M2Custom()   { SendCode(17, 2); }
void M3Custom()   { SendCode(18, 2); }
void M4Custom()   { SendCode(19, 2); }
void Moon1()      { SendCode(20, 2); }
void Moon2()      { SendCode(21, 2); }
void Moon3()      { SendCode(22, 2); }
void DawnDusk()   { SendCode(23, 2); }
void Cloud1()     { SendCode(24, 2); }
void Cloud2()     { SendCode(25, 2); }
void Cloud3()     { SendCode(26, 2); }
void Cloud4()     { SendCode(27, 2); }
void Storm1()     { SendCode(28, 2); }
void Storm2()     { SendCode(29, 2); }
void Storm3()     { SendCode(30, 2); }
void Storm4()     { SendCode(31, 2); }