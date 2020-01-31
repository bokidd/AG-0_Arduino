/*
  This is a example written for the SparkFun Qwiic Joystick
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15168

  Written by Wes Furuya @ SparkFun Electronics, February 5th, 2019

  The Qwiic Joystick is a I2C controlled analog joystick
  
  Example 4- Joystick Serial Output:
  This program uses the Qwiic Joystick Arduino Library to read the current
  joystick position and button state, which is then printed out as directions
  in the Serial Monitor.

  https://github.com/sparkfun/SparkFun_Qwiic_Joystick_Arduino_Library/examples

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "VernierLib.h" //include Vernier functions in this sketch
VernierLib Vernier; //create an instance of the VernierLib library
#include <Wire.h>
//#include "SparkFun_Qwiic_Joystick_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_joystick
#include <Wire.h>  // Include Wire if you're using I2C
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <OneWire.h> 
#include <SoftwareSerial.h>

//Major Classes
SoftwareSerial mySerial(8,9);
OpenLog myLog; //Create instance

//////////////////
//  Averaging   //
//////////////////
float tempAvg = 0;
float phAvg = 0;
float DOAvg = 0;
int countAvg = 0;

///////////////////
//  DS Temp Sensor //
///////////////////////
int DS18S20_Pin = 4; //DS18S20 Signal pin on digital 2
char tmpstring[10];

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

//////////////////////////
// Vernier Definition //
//////////////////////////

float dataRate = 2;             // set # of samples per second.
const char delimiter = '\t';  // delimitter character

const int ThermistorPIN = A0; // A0 for Analog1 and A2 for Analog 2
float Temp;
int rawAnalogReading;

byte muxlsb = 10;
byte muxmsb = 11;

// Variables used in the code for calculations
unsigned long timeRef;    // reference for starting time
unsigned long timerVal;    // reference for counting time

unsigned long timeInterval;
unsigned long ndx;        // index for data counter
unsigned long thermistor;
float rawCount1; //create global variable for reading from A/D converter (0-1023)
float voltage1; //create global variable for voltage (0-5V)
float sensorValue1; //create global variable for sensor value
float slope1 = 4.44; //create global variable for slope for a Dual-Range Force Sensor +/-10N range
float intercept1 = -0.44; //create global variable for intercept for a Dual-Range Force Sensor +/-10N range
String units1 = "mg/L";
float rawCount2; //create global variable for reading from A/D converter (0-1023)
float voltage2; //create global variable for voltage (0-5V)
float sensorValue2; //create global variable for sensor value
float slope2 = -4.23; //create global variable for slope for a Dual-Range Force Sensor +/-10N range
float intercept2 = 14.55; //create global variable for intercept for a Dual-Range Force Sensor +/-10N range
String units2 = "pH's";

int buttonPin = 12;
int ledPin = 13;
bool buttonState = 1;
bool prevButtonState = 1;
int buttonCounter = -1;

//////////////////////////
// MicroOLED Definition //
//////////////////////////
//The library assumes a reset pin is necessary. The Qwiic OLED has RST hard-wired, so pick an arbitrarty IO pin that is not being used
#define PIN_RESET 9  
//The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (Default), or set to 0 if it's closed.
#define DC_JUMPER 1 

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration
uint8_t Address = 0x20; //Start address (Default 0x20)

/////////////////////
// GPS Definitions //
/////////////////////
float GPSLat;
float GPSLon;

void setup() {
  //Serial.begin(9600); //setup communication to display
  delay(100);
  pinMode(muxlsb, OUTPUT);  // multiplexer pins for AutoID
  pinMode(muxmsb, OUTPUT);  // multiplexer pins for AutoID
  pinMode(7,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(buttonPin,INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(buttonPin,HIGH);
  digitalWrite(ledPin, LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  digitalWrite(5,HIGH);
  //Vernier.autoID(); //identify the sensor being used
  Wire.begin();
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000);     // Delay 1000 ms
  oled.clear(PAGE); // Clear the buffer.
  Serial.begin(57600);
  Serial.println("Qwiic Joystick Example");
  mySerial.begin(9600);
  randomSeed(analogRead(A0) + analogRead(A1));
  /*
  if(joystick.begin(Wire, Address) == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }
  */
  myLog.begin(); //Open connection to OpenLog (no pun intended)
  //myLog.println("This goes to the log file");
  Serial.println("This goes to the terminal");
  timeInterval = 1000 / dataRate;  // calculates the timeInterval based on # of samples per second.
  timeRef = millis();
  timerVal = millis();
}

void loop() {
    //Serial.println("got to loop");
    //Serial.print(" "); //print a space

    digitalWrite(muxlsb, LOW); //set multiplexer for BTA1
    digitalWrite(muxmsb, LOW);
    int countID = analogRead(A5);
    float VoltageID = countID / 1023.0 * 5;
    if(VoltageID > 4) // DO probe on BTA1, pH probe on BTA2
    {
      //Serial.println(Vernier.sensorUnits()); //print units and skip to next line
      rawCount1=analogRead(A0); //read one data value (0-1023)
      voltage1=rawCount1/1023*5; //convert raw count to voltage (0-5V)
      sensorValue1=slope1*voltage1+intercept1; //convert to sensor value with linear calibration equation
      rawCount2=analogRead(A2); //read one data value (0-1023)
      voltage2=rawCount2/1023*5; //convert raw count to voltage (0-5V)
      sensorValue2=slope2*voltage2+intercept2; //convert to sensor value with linear calibration equation
    }
    else if (VoltageID <4)//pH probe on BTA1, DO probe on BTA2
    {
      //Serial.println(Vernier.sensorUnits()); //print units and skip to next line
      rawCount1=analogRead(A2); //read one data value (0-1023)
      voltage1=rawCount1/1023*5; //convert raw count to voltage (0-5V)
      sensorValue1=slope1*voltage1+intercept1; //convert to sensor value with linear calibration equation
      rawCount2=analogRead(A0); //read one data value (0-1023)
      voltage2=rawCount2/1023*5; //convert raw count to voltage (0-5V)
      sensorValue2=slope2*voltage2+intercept2; //convert to sensor value with linear calibration equation
    }
    Temp = (getTempF()-32)*5/9;
    if(Temp > 212)
    {
      Temp = -1;
    }

    tempAvg += Temp;
    DOAvg += sensorValue1;
    phAvg += sensorValue2;
    countAvg++;
    
    oled.clear(PAGE);            // Clear the display
    oled.setCursor(0, 0);        // Set cursor to top-left
    oled.setFontType(0);         // Smallest font
    oled.print("T : ");          // Print "A0"
    oled.setFontType(1);         // 7-segment font
    //oled.setCursor(8,0);
    oled.print(Temp,1);  // Print a0 reading
    oled.setCursor(0, 16);       // Set cursor to top-middle-left
    oled.setFontType(0);         // Repeat
    oled.print("DO: ");
    oled.setFontType(1);
    oled.print(sensorValue1,2);
    oled.setCursor(0, 32);
    oled.setFontType(0);
    oled.print("pH: ");
    oled.setFontType(1);
    oled.print(sensorValue2,2);
    oled.display();

    //mySerial.println("HB_GPS");
    char recieved;
    String piString;
    bool newData = false;
    while(mySerial.available())
    {
      recieved = mySerial.read();
      piString += recieved;
      newData = true;
    }
    if(newData) 
    {
      //Serial.print(piString);
      int  ID1 = piString.indexOf(',');          //After LAT
      int  ID2 =piString.indexOf(',', ID1 + 1);  //After LatNum
      int  ID3 =piString.indexOf(',', ID2 + 1);  //After LON
      String latString = piString.substring(ID1+1,ID2);
      String lonString = piString.substring(ID3+1);
      GPSLat = latString.toFloat();
      GPSLon = lonString.toFloat();
      //Serial.print("lit: ");
      //Serial.print(GPSLat,6);
      //Serial.print(", lig: ");
      //Serial.println(GPSLon,6);
    }

// DATA LOGGING IS ONLY DONE AT INTERVALS! //
  if ((millis()) >= timeInterval + timerVal)  // controls so only runs once per timeInterval
  {
    timerVal = millis();
    //ndx++;  
    /*
    //the print below does the division first to avoid overflows
    Serial.print((float)(millis() - timeRef) / 1000, 2); 
    rawAnalogReading = analogRead(ThermistorPIN);  // reads raw analog value from Arduino
    thermistor = resistance(rawAnalogReading);     // converts raw analog value to a resistance
    Temp = steinharthart(thermistor);              // Applies the Steinhart-hart equation
    */

    if (prevButtonState == 1)
    {
      prevButtonState = 0;
      buttonCounter++;
    }
    digitalWrite(ledPin,HIGH);

    double timeDouble = millis()-timeRef;
    timeDouble = timeDouble/1000;
    String timeString = String(timeDouble,1);
    //Serial.print(delimiter); //tab character
    Serial.print("time, "+timeString);   // display temperature to one digit
    Serial.print(", T, "+String(Temp));
    Serial.print(", DO, "+String(sensorValue1));
    Serial.print(", pH, "+String(sensorValue2));
    Serial.print(", WPT, "+String(buttonCounter));
    Serial.print(", LAT, "+String(GPSLat,6));
    Serial.println(", LON, "+String(GPSLon,6));
/*
    myLog.append("dataLog.txt");
    myLog.print("time: "+String(millis()));
    myLog.print(", T: "+String(tempAvg/countAvg));
    myLog.print(", DO: "+String(DOAvg/countAvg));
    myLog.println(", pH: "+String(phAvg/countAvg));
    myLog.syncFile();                                
*/
    myLog.append("dataLog.txt");
    myLog.print("time: "+String(millis()/1000));
    myLog.print(", T: "+String(Temp));
    myLog.print(", DO: "+String(sensorValue1));
    myLog.print(", pH: "+String(sensorValue2));
    myLog.print(", WPT: "+String(buttonCounter));
    myLog.print(", LAT, "+String(GPSLat,6));
    myLog.println(", LON, "+String(GPSLon,6));
    myLog.syncFile();                                

    tempAvg = 0;
    DOAvg = 0;
    phAvg = 0;
    countAvg = 0;
  }
  else
  {
    digitalWrite(ledPin,LOW);
  }


  //delay(200);
}

float getTempF(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return (TemperatureSum * 18 + 5)/10 + 32;
}
