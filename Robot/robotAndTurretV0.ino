#include <Servo.h>

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

int potPin3 = A2, potPin4 = A3;
int defPosThr,defPosStr;

int servoPin1 = 10, servoPin2 = 11; // servo motors pins
int potPin1 = A0, potPin2 = A1;   // potentiometers pins
Servo servo1, servo2;             // servo motor "handlers"

int laserPin = 8;
int buttonPin = 12;

int xPos = 90, yPos = 90;
int defPosX, defPosY;

int getValidRead(int pin, int valid);

void setup() {
  Serial.begin(9600);
  // set all the motor control pins to outputs
//  pinMode(enA, OUTPUT);
//  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT); 
  pinMode(potPin3, INPUT);
  pinMode(potPin4, INPUT);
    
  // Configure the 2 pot pins as input 
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(laserPin, OUTPUT);

  // something here is quite ODD
  defPosX = analogRead(potPin1);
  defPosY = analogRead(potPin2);

  defPosThr = analogRead(potPin3);
  defPosStr = analogRead(potPin4);
  
  // Connect to the 2 servo motors
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
    


  digitalWrite(laserPin, HIGH);

  servo1.write(xPos);
  servo2.write(yPos);

  Serial.print("defPosX "); 
  Serial.print(defPosX);
  Serial.print(" defPosY "); 
  Serial.println(defPosY);
}

void loop()
{
  int reading;
  int increment;
  int readVal;
  
  reading = analogRead(potPin3);
  if(reading > (defPosThr + defPosThr/10)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(reading < (defPosThr - defPosThr/10)) {
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

  reading = analogRead(potPin4);
  if(reading > (defPosStr + defPosStr/10)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if(reading < (defPosStr - defPosStr/10)) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  readVal = analogRead(potPin1);     // 0 to 1023
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

  readVal = analogRead(potPin2);     // 0 to 1023  
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

