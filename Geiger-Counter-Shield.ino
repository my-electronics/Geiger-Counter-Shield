/* GEIGER COUNTER SHIELD FOR ARDUINO V2.1 

Author: My Electronics http://my-electronics.net
Copyright (C) 2019 My Electronics

For more information and to find out where to buy the kit visit http://my-electronics.net

SAFTEY NOTES:
This circuit design includes a switch‐mode voltage converter which can generate 600 VDC.
DO NOT USE IF YOU DON’T KNOW HOW TO HANDLE HIGH VOLTAGES. 
You are fully responsible for your safety during the assembly and operation of this 
device. We do not guarantee that the radiation readings you may see, or may not see, on 
the display are correct. You are fully responsible for your safety and health in high 
radiation areas. 

The source code can be modified for personal and educational purposes. 
The commercial usage is restricted. 

*/

//--------------------------------- Required libaries ---------------------------------// 
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// ------------------------------------- Defines --------------------------------------//

#define LED1 A1                     // LED for events on pin A1
#define LED2 A2                     // LED for warnings on pin A2
#define BUZZER A3                   // Buzzer on pin A3
#define BUTTON1 A4                  // Button to change the display state on pin A4
#define BUTTON2 A5                  // Button to mute the buzzer on pin A5

#define PWM_INIT 29                 // Initial PWM value for 400 V
#define ADC_SET 372                 // ADC setpoint for 400 V 

#define CPS_WARNING 1000            // 1000 CPS warning level

#define DOSE_RATE_FACTOR 0.0057     // Factor to convert from CPM to dose rate in µSv/h 

#define SD_LOGGER false
#define CS 3                        // Set chip select pin for SD card
   
#define SERIAL_LOGGER true 
                  
//---------------------------------- Global variables ---------------------------------//

LiquidCrystal lcd(9, 8, 7, 6, 5, 4);

int button1State = 1;                 // Button status

static int displayStatus = 0;         // Default display status CPS + bar graph
static int settingsState = 0;         // Default display status CPS + bar graph
int buzzerStatus = 1;                 // Buzzer status on startup is ON
int buzzerFrequency = 4000;           // Buzzer ON is 4 kHz, Buzzer OFF is 0 Hz


boolean event = false;                // Flag for GM tube event 

static unsigned int cps;              // CPS
static unsigned long cp10s;           // Counts per 10 s
static unsigned long cp10sValues[] = {0, 0, 0, 0, 0, 0};  
static unsigned long cpm;             // CPM
static unsigned long cpmPrev;         // Previous CPM value
static unsigned long cpmDiff;

int n = 0;                            // Counter for array
static float doseRate;                // Variable for the dose rate

int pwm = PWM_INIT;
int adc_val = 0;                           // Variable to store HV reading

unsigned long t = 0;                  // Counters for timing 
unsigned long timer1 = 0;
unsigned long timer2 = 0;
unsigned long timer3 = 0;

int sdState = 0;
char fileName[20];
int fileCount = 0;

/////////////////////////////////////////////////////////////////////////////////////////
//                                        SETUP                                        //
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Initialize global variables
  cps = 0;
  cp10s = 0;
  cpm = 0;

  // Configure I/O pins
  pinMode(LED1, OUTPUT);           
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);           
  digitalWrite(LED2, LOW);
  pinMode(BUZZER, OUTPUT);         
  digitalWrite(BUZZER, LOW);
  
  pinMode(BUTTON1, INPUT);        // Set buttons pins as input with internal pullup
  digitalWrite(BUTTON1, HIGH);
  pinMode(BUTTON2, INPUT);
  digitalWrite(BUTTON2, HIGH);  

  
  // Start LCD display  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print(" Geiger Counter ");
  lcd.setCursor(0, 1);
  lcd.print("     Shield     ");

  delay(2500);

  // --------------------- Setup the SD card for CPM logging ------------------------- //

  #if (SD_LOGGER)
    if (!SD.begin(CS)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("No SD Card");
      lcd.setCursor(0, 1);
      lcd.print("Detected");
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("SD Card Detected");
      sdState = 1;
      sprintf(fileName, "Data_000.txt");
      while(SD.exists(fileName) && (fileCount < 1000)) {
        fileCount++;
        sprintf(fileName, "Data_%03d.txt", fileCount);
      }
      lcd.setCursor(0, 1);
      lcd.print(fileName);
    }
  
    delay(2500);
  
  #endif

  // ------------------------ Get display statw from EEPROM -------------------------- // 
  getDisplayStatus();               
  setDisplayStatus(displayStatus);

  // -------------------- Setup the serial port for CPM logging ---------------------- //
  
  Serial.begin(9600);            
  delay(100);

  // -------------------------- Setup the HV for the tube ---------------------------- //
  
  pinMode(10, OUTPUT);                   // Set output pin for PWM  
  pinMode(A0, INPUT);                    // Set analog input for HV measure

  // Set reference voltage to the internal 1.1 V  
  // Set PWM frequency of pin 10 to 4 kHz

  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)   // For Arduino UNO    
    analogReference(INTERNAL);           
    TCCR1B = TCCR1B & 0b11111000 | 0x02;   
  #else                                                           // For Arduino MEGA                     
    analogReference(INTERNAL1V1);          
    TCCR2B = TCCR2B & 0b11111000 | 0x02;   
  #endif

  delay(50);     
      
  // ------------ Configure interrupt on INT0 for detecting tube events -------------- //
  
  pinMode(2, INPUT);                     // Set pin INT0 as input (Hardware 10 k pull-up)
  attachInterrupt(0, tubeEvent, FALLING);  
  interrupts();
}
/////////////////////////////////////////////////////////////////////////////////////////
//                                      END SETUP                                      //
/////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////
//                                        LOOP                                         //
/////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  t = millis();

  if (event == true){                    // Make LED blink and buzzer beep for 1 ms
    PORTC |= _BV (1);                    // digitalWrite (LED1, HIGH);
    tone(BUZZER, buzzerFrequency, 1);    // Tone for 1 ms
    PORTC &= ~_BV (1);                   // digitalWrite (LED1, LOW); 
    event = false;
  }
   
  //---------------------------- What to do every 5 ms ---------------------------------//
  
  if (t - timer1 > 5) { // Check and correct tube voltage if needed        
    timer1 = t;
    
    adc_val = analogRead(0); // Read the A0 pin
      
    if (adc_val < ADC_SET && adc_val < 558) // Limit maximum voltage to 600 V
      pwm++;   
    else if (adc_val > ADC_SET)
      pwm--;
    
    analogWrite(10, pwm);  
  }
    
  //----------------------------- What to do every 1 s --------------------------------//
  
  if(t - timer2 > 1000) { // Check if button was pressed 
    timer2 = t;
    
    if (!digitalRead(BUTTON1)) { // Change display CPM <-> CPS
        displayStatus = 1 - displayStatus; 
        setDisplayStatus(displayStatus);     
    }
    
    if (displayStatus == 0) { // Display CPS and bargraph if the CPS display is selected
      clearDisplay(0, 0, 5);
      lcd.setCursor(0, 0);
      if (cps > 10000) // Limit maximum possible CPS
        cps = 10000;
      if (cps > CPS_WARNING) // Lit LED2 for warning
        PORTC |= _BV (1);
      else 
        PORTC &= ~_BV (1);
      lcd.print(cps);
      drawBars();
    }

    if (!digitalRead(BUTTON2)) { // Toggle buzzer
      buzzerStatus = 1 - buzzerStatus; 
      if (buzzerStatus == 1) {
        buzzerFrequency = 4000;
      }  
      else {
        buzzerFrequency = 0;
      }
    }
    
    cps = 0;
  }

  //---------------------------- What to do every 10 s ------------------------------//
  
  if (t - timer3 > 10000) { 
    timer3 = t;
    
    cp10sValues[n] = cp10s;
    cpmPrev = cpm;

    cpm = 0;
    for (int i = 0; i < 6; i++)
      cpm += cp10sValues[i];

    if (n < 6)
      n++;
    else
      n = 0;

    // Check if the CPM level changes rapidly 
    cpmDiff = cpmPrev - cpm;
    if (abs(cpmDiff) >= 50) { // 50 is an empirical value                           
      cpm = cp10s * 6;
    }

    if (displayStatus == 1) { // Display CPM and dose rate if the CPM display is selected
      // Calculate dose rate 
      doseRate = cpm * DOSE_RATE_FACTOR;

      // Print CPM and dose rate on LCD
      clearDisplay(0, 1, 6);
      lcd.setCursor(0, 1);
      lcd.print(cpm);

      clearDisplay(0, 0, 7);
      lcd.setCursor(0, 0);
      printFloat(doseRate);
    }

   #if (SERIAL_LOGGER)
     Serial.println(cpm); // Send the CPM data via the serial port
   #endif

   #if (SD_LOGGER)
     if (sdState) {
       File file = SD.open(fileName, FILE_WRITE);
       if (file) {
         file.println(cpm); // Write the CPM data to file on SD card
         file.close();
       }
     }
   #endif
   
   cp10s = 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////
//                                      LOOP END                                       //
/////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////
//                                   SUB PROCEDURES                                    //
/////////////////////////////////////////////////////////////////////////////////////////

//-------------------------- Process tube event interrupt -----------------------------//
void tubeEvent ()           
{ 
  cps++;         // Increase CPS counter
  cp10s++;       // Increase counter for 10 seconds counter
  event = true;  // Set event flag true
}

//---------------------------------- Draw bargraph ------------------------------------//
void drawBars() 
{ 
  int n = (int)(log10(cps) - 0.204); // 0.204 = log10(10/16)
  int bars = (int)cps/pow(10, n);
  
  //int bars = cps;
  //if (cps > 16)
  //  bars = 16;
  lcd.setCursor(0, 1);
  for (int i = 0; i < bars; i++) {
    lcd.write(0xff);                     
  }
  for (int i = bars; i < 16; i++) {
    lcd.write('-');                     
  }
}


//-------------------------------- Print float on LCD --------------------------------//
void printFloat(float val)
{        
  lcd.print(int(val));                      
  lcd.write('.');                        
  if ((val - int(val)) < 0.1) {
    lcd.write('0');
    lcd.print(int(100*(val - int(val))));
  }
  else {
    lcd.print(int(100*(val - int(val))));
  }
}


//-------------------------------- Clear LCD zone -------------------------------------//
void clearDisplay(byte x, byte y, byte zone)
{
  int i;
  lcd.setCursor (x, y);
  for (i = 0; i < zone; i++){
    lcd.write(' ');
  }
}


//------------------------ Store LCD display status in EEPROM ----------------------//
void setDisplayStatus(int ds)
{  
  if (ds == 0) {
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print("CPS:"); 
    cps = 0;
    EEPROM.write(0x00, 0x00);
    displayStatus = 0;
  }
  else {
    lcd.clear();
    lcd.setCursor(11, 0);
    lcd.write("\xe4"); // µ              
    lcd.write("Sv/h");
    lcd.setCursor(13, 1);
    lcd.print("CPM"); 
    cpm = 0;
    for (int i = 0; i < 6; i++) 
      cp10sValues[i] = 0;
    EEPROM.write(0x00, 0x01);
    displayStatus = 1;
  }
  
}

//-------------------------- Get display status from EEPROM ---------------------------//
void getDisplayStatus()
{
  byte ds = EEPROM.read(0x00);
  if (ds == 1) {
    displayStatus = 1;
  }
  else {
    displayStatus = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//                                SUB PROCEDURES END                                   //
/////////////////////////////////////////////////////////////////////////////////////////
  
