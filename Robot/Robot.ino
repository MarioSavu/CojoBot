#include <Servo.h>
#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)
// NEED the SoftwareServo library installed
// http://playground.arduino.cc/uploads/ComponentLib/SoftwareServo.zip
#include <SoftwareServo.h>  // Regular Servo library creates timer conflict!

/* - CONNECTIONS:
   - nRF24L01 Radio Module: See http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
  - V2.12 02/08/2016
   - Uses the RF24 Library by TMRH20 and Maniacbug: https://github.com/TMRh20/RF24 (Download ZIP)
   Questions: terry@yourduino.com */
#define  CE_PIN  7   // The pins to be used for CE and SN
#define  CSN_PIN 8

#define POT_MID_VAL 511
// connect motor controller pins to Arduino digital pins
// motor one
int enA = 0;
int in1 = 2;
int in2 = 3;
// motor two
int enB = 1;
int in3 = 4;
int in4 = 5;
int laserPin = 6;
int servoPin1 = 9, servoPin2 = 10; // servo motors pins

/*-----( Declare objects )-----*/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus (usually) pins 7 & 8 (Can be changed) */
RF24 radio(CE_PIN, CSN_PIN);
SoftwareServo servo1, servo2;             // servo motor "handlers"

int defPosThr,defPosStr;
int xPos = 90, yPos = 90;
int defPosX, defPosY;
byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

struct dataStruct {
  unsigned long _micros;  // to save response times
  int Xposition;          // The Joystick position values
  int Yposition;
  int X2position;
  int Y2position;
//  bool switchOn;          // The Joystick push-down switch
} myData; 

void processMovement();

void stopRobotMovement() 
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup() {
  Serial.begin(115200);   // set all the motor control pins to outputs
//  pinMode(enA, OUTPUT);
//  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT); 
    
  pinMode(laserPin, OUTPUT);

  defPosX = POT_MID_VAL;
  defPosY = POT_MID_VAL;
  defPosThr = POT_MID_VAL;
  defPosStr = POT_MID_VAL;
  
  // Connect to the 2 servo motors
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
    
  digitalWrite(laserPin, HIGH);

  servo1.write(xPos);
  servo2.write(yPos);

  radio.begin();          // Initialize the nRF24L01 Radio
  radio.setChannel(108);  // 2.508 Ghz - Above most Wifi Channels
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range
  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //   radio.setPALevel(RF24_PA_MAX);
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();

  Serial.print("defPosX "); 
  Serial.print(defPosX);
  Serial.print(" defPosY "); 
  Serial.println(defPosY);
}

void loop() 
{
  SoftwareServo::refresh();//refreshes servo to keep them updating
  
  if ( radio.available())
  {
    
    while (radio.available())   // While there is data ready to be retrieved from the receive pipe
    {
      radio.read( &myData, sizeof(myData) );             // Get the data
    }

    radio.stopListening();                               // First, stop listening so we can transmit
    radio.write( &myData, sizeof(myData) );              // Send the received data back.
    radio.startListening();                              // Now, resume listening so we catch the next packets.

    Serial.print(F("Packet Received - Sent response "));  // Print the received packet data
    Serial.print(myData._micros);
    Serial.print(F("uS X= "));
    Serial.print(myData.Xposition);
    Serial.print(F(" Y= "));
    Serial.print(myData.Yposition);
    Serial.print(F(" X2= "));
    Serial.print(myData.X2position);
    Serial.print(F(" Y2= "));
    Serial.println(myData.Y2position);
    
  } // END radio available
  else 
  {
    stopRobotMovement();
  }
  //processMovement();
}
void processMovement()
{
  int increment;
  int readVal;
  
  readVal = myData.X2position;
  if(readVal > (defPosThr + defPosThr/10)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(readVal < (defPosThr - defPosThr/10)) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  readVal = myData.Y2position;
  if(readVal > (defPosStr + defPosStr/10)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if(readVal < (defPosStr - defPosStr/10)) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  readVal = myData.Xposition;     // 0 to 1023
  Serial.print(" potPin1: ");
  Serial.print(readVal);
  if(readVal < (defPosX - defPosX/10)) {
    increment = map(readVal, defPosX, 0, 0, 10);
    xPos -= increment;
    if(xPos < 0)  xPos = 0;
  }
  else if(readVal > (defPosX + defPosX/10)) {
    increment = map(readVal, defPosX, 1024, 0, 10);
    xPos += increment;
    if(xPos > 180)  xPos = 180;
  }  
  Serial.print(" xIncr: ");
  Serial.print(increment);
  servo1.write(xPos);

  readVal = myData.Yposition;     // 0 to 1023  
  Serial.print(" potPin1: ");
  Serial.print(readVal);
  if(readVal < (defPosY - defPosY/10)) {
    increment = map(readVal, defPosY, 0, 0, 10);
    yPos -= increment;
    if(yPos < 0)  yPos = 0;
  }
  else if(readVal > (defPosY + defPosY/10)) {
    increment = map(readVal, defPosY, 1024, 0, 10);
    yPos += increment;
    if(yPos > 180)  yPos = 180;
  }  
  Serial.print(" yIncr: ");
  Serial.print(increment);
  servo2.write(yPos);
  
  Serial.print(" xPos: ");
  Serial.print(xPos);
  Serial.print(" yPos: ");
  Serial.println(yPos);
  delay(5);
}

