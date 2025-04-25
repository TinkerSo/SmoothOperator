//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

// Variables for ultrasonic
float Inch1=0.00;
float cm1=0.00;
float Inch2=0.00;
float cm2=0.00;
float Inch3=0.00;
float cm3=0.00;
float Inch4=0.00;
float cm4=0.00;
float Inch5=0.00;
float cm5=0.00;
int SonarPin1=A0;
int SonarPin2=A1;
int SonarPin3=A2;
int SonarPin4=A3;
int SonarPin5=A4;
int sensorValue1;
int sensorValue2;
int sensorValue3;
int sensorValue4;
int sensorValue5;

float threshold = 20;

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);

  // Attach ultrasonic inputs
  pinMode(SonarPin1,INPUT);
  pinMode(SonarPin2,INPUT);
  pinMode(SonarPin3,INPUT);
  pinMode(SonarPin4,INPUT);
  pinMode(SonarPin5,INPUT);

  Serial.begin(9600);
}

void loop() {
  // Get ultrasonic sensor values
  sensorValue1=analogRead(SonarPin1);
  sensorValue2=analogRead(SonarPin2);
  sensorValue3=analogRead(SonarPin3);
  sensorValue4=analogRead(SonarPin4);
  sensorValue5=analogRead(SonarPin5);
  delay(50);
  Inch1= (sensorValue1*0.497);
  cm1=Inch1*2.54;

  Inch2= (sensorValue2*0.497);
  cm2=Inch2*2.54;

  Inch3= (sensorValue3*0.497);
  cm3=Inch3*2.54;

  Inch4= (sensorValue4*0.497);
  cm4=Inch4*2.54;

  Inch5= (sensorValue5*0.497);
  cm5=Inch5*2.54;
  Serial.print(cm1);
  Serial.print("cm1 ");
  Serial.print(cm2);
  Serial.print("cm2 ");
  Serial.print(cm3);
  Serial.print("cm3 ");
  Serial.print(cm4);
  Serial.print("cm4 ");
  Serial.print(cm5);
  Serial.println("cm5 ");
  
//  roboclaw.ForwardM1(address,64); //start Motor1 forward at half speed
//  roboclaw.BackwardM2(address,64); //start Motor2 backward at half speed
//  delay(2000);
//
//  roboclaw.BackwardM1(address,64);
//  roboclaw.ForwardM2(address,64);
//  delay(2000);

//  roboclaw.ForwardBackwardM1(address,127); //start Motor1 forward at half speed
//  roboclaw.ForwardBackwardM2(address,0); //start Motor2 backward at half speed
//  delay(2000);
//
//  roboclaw.ForwardBackwardM1(address,64); //start Motor1 forward at half speed
//  roboclaw.ForwardBackwardM2(address,64); //start Motor2 backward at half speed
//  delay(2000);
//
//  roboclaw.ForwardBackwardM1(address,0);
//  roboclaw.ForwardBackwardM2(address,127);
//  delay(2000);


  if(cm1 > threshold and threshold and threshold and cm3 > threshold and cm4 > threshold){ 
    roboclaw.ForwardBackwardM1(address,127); //start Motor1 forward at half speed
    roboclaw.ForwardBackwardM2(address,127); //start Motor2 backward at half speed
  }
  else{
    roboclaw.ForwardBackwardM1(address,64); //start Motor1 forward at half speed
    roboclaw.ForwardBackwardM2(address,64); //start Motor2 backward at half speed
  }
  delay(100);
  
}
