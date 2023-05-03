#include <dht11.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <uRTCLib.h>


#define RDA 0x80
#define TBE 0x20


// UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *) 0xC0;
volatile unsigned char *myUCSR0B  = (unsigned char *) 0xC1;
volatile unsigned char *myUCSR0C  = (unsigned char *) 0xC2;
volatile unsigned int  *myUBRR0   = (unsigned int  *) 0xC4;
volatile unsigned char *myUDR0    = (unsigned char *) 0xC6;
// Timer Pointers
volatile unsigned char *myTCCR1A  = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6f;
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned int  *) 0x84;
// ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
// Digital Pins
volatile unsigned char* PORT_B = (unsigned char*) 0x25;
volatile unsigned char* DDR_B  = (unsigned char*) 0x24;
volatile unsigned char* PIN_B  = (unsigned char*) 0x23;


// Serial
#define BAUD 9600


// Stepper
#define STEPS_PER_REVOLUTION 2038 // 32 steps but has a 63.68395:1 gearbox inside
#define STEPPER_SPEED 15 // RPMs

#define STEPPER_IN1 2
#define STEPPER_IN2 3
#define STEPPER_IN3 4
#define STEPPER_IN4 5

#define STEP_RATIO_FOR_VENTS 2.7

#define POTENTIOMETER_ANALOG_PIN 1
#define VENT_ADC_TOLERANCE 100

#define ANALOG_MAX 1024

Stepper ventStepper(STEPS_PER_REVOLUTION, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);
unsigned short ventStepperPositition;


// Humidity
#define HUMIDITY_SENSOR 6
unsigned char temperature;
unsigned char humidity;

dht11 DHT;


// LCD
#define LCD_RS  23
#define LCD_RW  25
#define LCD_E   27
#define LCD_D0  29
#define LCD_D1  31
#define LCD_D2  33
#define LCD_D3  35
#define LCD_D4  37
#define LCD_D5  39
#define LCD_D6  41
#define LCD_D7  43

#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
bool updateDisplay = true;

// Timers
#define TIMER_STOP  0xF8
#define TIMER_START 0x04 // 256 Prescaling
#define TICKS_PER_SECOND 62500 // Not enough ticks to delay 1 minute even wtih 1024 presclaing
#define SECONDS_PER_UPDATE 60
volatile unsigned char seconds = 0; // Keeps track of how many times the timer has ticked to update display once per minute as per directions


// Fan
#define FAN 0


// Water Level Sensor
#define WATER_LEVEL_SENSOR 0 // Analog
unsigned short waterLevel;


// RTC
#define RTC_I2C_ADDRESS 0x68
uRTCLib rtc(RTC_I2C_ADDRESS);


// Buttons
#define START_STOP_BUTTON 2 // 2 On PORT_B has an interrupt vector
#define RESET_BUTTON 0

// LEDs (on PORT_B)
#define OFF 0
#define ON 1
#define GREEN_LED  0
#define YELLOW_LED 0
#define BLUE_LED   0
#define RED_LED    0

// Thresholds
#define TEMPERATURE_TARGET 20
#define TEMPERATURE_HIGH  25
#define WATER_LEVEL_THRESHOLD 200

// Cooler States
#define DISABLED 0
#define IDLE     1
#define RUNNING  2
#define ERROR    3

volatile unsigned char coolerState;


void setup()
{

  // Serial  
  U0Init(BAUD);

  // Stepper
  adc_init();

  ventStepper.setSpeed(STEPPER_SPEED);
  ventStepperPositition = adc_read(POTENTIOMETER_ANALOG_PIN); // This sets the "default" position of the vents to whatever it was on startup

  // LCD
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.noCursor();
  lcd.display();
  setup_timer_regs();

  // RTC
  URTCLIB_WIRE.begin();
  // LEAVE COMMENTED
  // rtc.set(10, 59, 16, 2, 1, 5, 23);

  // Start button
  *ddr_b &= ~(1 << START_STOP_BUTTON);

  // LEDs
  *ddr_b |= 1 << GREEN_LED;
  *ddr_b |= 1 << YELLOW_LED;
  *ddr_b |= 1 << BLUE_LED;
  *ddr_b |= 1 << RED_LED;


  // Starting State
  coolerState = DISABLED;
}



void loop()
{
  // Always update the clock
  rtc.refresh();

  if (updateDisplay) {
    displayTemperatureData();
    updateDisplay = false;
  }


  switch (coolerState)
  {
    case DISABLED:
      stopDisplayRefreshTimer();
      // Yellow LED on 
      setLED(RED, OFF);
      setLED(GREEN, OFF);
      setLED(BLUE, OFF);
      setLED(YELLOW, ON);
      lcd.print("DISABLED");
      adjustVents();
      break;


    case IDLE:
      // Green LED on 
      setLED(RED, OFF);
      setLED(GREEN, ON);
      setLED(BLUE, OFF);
      setLED(YELLOW, OFF);

      setPin(FAN, OFF);
      adjustVents();
      // Temperature data is updated every minute from timer ISR
      if (temperature >= TEMPERATURE_HIGH) {
        printString(getDateString() + "    TEMPERATURE TOO HOT - FAN TURNING ON");
        coolerState = RUNNING

      } else if (adc_read(WATER_LEVEL_SENSOR) <= WATER_LEVEL_THRESHOLD) {
        printString(getDateString() + "    ERROR: WATER LEVEL TOO LOW");
        coolerState = ERROR;
      }
      break;


    case RUNNING:
      // Blue LED on 
      setLED(RED, OFF);
      setLED(GREEN, OFF);
      setLED(BLUE, ON);
      setLED(YELLOW, OFF);

      setPin(FAN, ON);
      adjustVents();

      // Temperature data is updated every minute from timer ISR
      if (temperature <= TEMPERATURE_TARGET) {
        printString(getDateString() + "    TARGET TEMPERATURE REACHED - FAN TURNING OFF");
        coolerState = IDLE;
        
      } else if (adc_read(WATER_LEVEL_SENSOR) <= WATER_LEVEL_THRESHOLD) {
        printString(getDateString() + "    ERROR: WATER LEVEL TOO LOW");
        coolerState = ERROR;
      }
      break;


    case ERROR:
      // Red LED on
      setLED(RED, ON);
      setLED(GREEN, OFF);
      setLED(BLUE, OFF);
      setLED(YELLOW, OFF);

      stopDisplayRefreshTimer();
      lcd.print("ERROR");

      setPin(FAN, OFF);
      if (isButtonPressed(RESET_BUTTON)) {
        if (adc_read(WATER_LEVEL_SENSOR) > WATER_LEVEL_THRESHOLD) {
          coolerState = IDLE;
          printString(getDateString() + "    WATER LEVEL VALID, RETURNING TO IDLE");
          startDisplayRefreshTimer();
        }
      }
      break;
  }
}


void adjustVents()
{
  unsigned short potVal = adc_read(POTENTIOMETER_ANALOG_PIN);
  short stepsToMove = potVal - ventStepperPositition;

  if (stepsToMove < -VENT_ADC_TOLERANCE || VENT_ADC_TOLERANCE < stepsToMove) {
    ventStepperPositition = potVal;
    ventStepper.step(stepsToMove/STEP_RATIO_FOR_VENTS);
  }
}


void updateAndDisplayTemperatureData()
{
  DHT.read(HUMIDITY_SENSOR);
  temperature = DHT.temperature;
  humidity = DHT.humidity;

  String temperatureString = "Temperature: " + String(temperature) + "C";
  String humidityString = "Humidity: " + String(humidity) + "%";
  lcd.clear();
  lcd.print(temperatureString);
  lcd.setCursor(0, 1);
  lcd.print(humidityString);
}



void printString(String str)
{
  for (unsigned int i = 0; i < str.length(); i++)
  {
    putChar(str.charAt(i));
  }
}


bool isButtonPressed(unsigned char button)
{
  return *PIN_B & (1 << button);
}


bool setPin(unsigned char pin, bool onOff)
{
  if (onOff){
    *PORT_B |= 1 << pin;
  } else {
    *PORT_B &= ~(1 << pin);
  }
}


void startDisplayRefreshTimer()
{
  *myTCCR1B |= TIMER_START;
}


void stopDisplayRefreshTimer()
{
  *myTCCR1B |= TIMER_STOP;
}


String getDateString()
{

  String date = "";
  date += rtc.month();
  date += "/";
  date += rtc.day();
  date += "/";
  date += rtc.year();

  date += "    ";

  date += rtc.hour();
  date += ":";
  date += rtc.minute();
  date += ":";
  date += rtc.second();

  return date;
}


// Start button ISR
ISR(PCINT2)
{
  printString("BUTTON PRESSED");
  if (coolerState == DISABLED)
  {
    coolerState = IDLE;
    printString(getDateString() + "    START BUTTON PRESSED: GOING TO IDLE");
    startDisplayRefreshTimer();
  } else {
    printString(getDateString() + "    STOP BUTTON PRESSED: GOING TO DISABLED");
    coolerState = DISABLED;
  }

}

// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &= TIMER_STOP;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (TICKS_PER_SECOND));
  // Serial.print("TIMER");
  // Start the Timer
  *myTCCR1B |= TIMER_START;

  seconds++;
  if (seconds >= SECONDS_PER_UPDATE) {
    seconds = 0;
    updateDisplay = true;
  }
}


void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}


unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}


void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}


unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}


unsigned char getChar()
{
  return *myUDR0;
}


void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}



void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x00000001;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x00000001;

  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (TICKS_PER_SECOND));
  *myTCCR1B |= TIMER_START;
}


