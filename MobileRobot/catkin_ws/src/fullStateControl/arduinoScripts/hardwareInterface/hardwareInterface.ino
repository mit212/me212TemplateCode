#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float32MultiArray.h>  
#include <std_msgs/Float32.h>  

#include <SPI.h>
#include "Encoder.h"
//#include "TimerOne.h"

// define some variables for the encoders
float motorEncData = 0;
float motorEncPos = 0;
float armEncData = 0;
float armEncPos = 0;
float calibArmEnc = 0;

// previous enc values
float motorEncPosOld = 0;
float armEncPosOld = 0;

// enc velocities
float motorEncVel = 0;
float armEncVel = 0;

// controller
float controlEffort = 0;
float umax = 1200; //q = 100
//float umax = 1500; //q = 1000
//float umax = 55; //q = 1000, with zeros
float u = 0;

// ros stuff
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> hardwareInterface;//ros::NodeHandle hardwareInterface;
std_msgs::Float32MultiArray encSignals;
ros::Publisher encPub("encPub", &encSignals);

//std_msgs::Float32 testData;
//ros::Publisher testPub("testPub", &testData);

void callback(const std_msgs::Float32& uData)
{
//  testData.data = float(1);
//  testPub.publish(&testData);

  // send command to motor
  float u = uData.data;
  int remap = 10;
  controlEffort = round( min( abs(u), umax ) / umax * (255 - remap)) + remap;
  
  if (u > -0.001) {
    digitalWrite(4, HIGH);
  }
  else if (u < 0.001) {
    digitalWrite(4, LOW);
  }
  else {
    analogWrite(5, 0);
  }
  analogWrite(5, controlEffort);
}

// subscribe to uPub
ros::Subscriber<std_msgs::Float32> uSub("uPub", &callback);

void setup()
{  
  // initialize ros
  hardwareInterface.initNode();

  // subscribe
  hardwareInterface.subscribe(uSub);

  // define message and advertise
  encSignals.layout.dim[0].size = 4;
  encSignals.data = (float *)malloc(sizeof(float)*4);
  encSignals.data_length = 4;
  hardwareInterface.advertise(encPub);
  
  // initialize encoders
  initEncoders();
  clearEncoderCount();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // do first read from encoders to zero them out
  motorEncData = readEncoder(1);
  motorEncPos = motorEncData * 360 / (64 * 26.9 * 4);

  delay(1);

  armEncData = readAS5147P(2);
  armEncData = armEncData * 360 / 16383;
  calibArmEnc = armEncData; //we need to subtract this off every time to zero out arm enc
  armEncPos = (armEncData - calibArmEnc) * (-1);

//  // initialize interrupt
//  Timer1.initialize(dtInterrupt);  /* Unit:Microsecond */
//  Timer1.attachInterrupt(readEncoders);
}


void loop()
{
  readEncoders();
  hardwareInterface.spinOnce();
}

void readEncoders()
{
  // read in from encoders and assign to x
  // motor position
  motorEncData = readEncoder(1);
  motorEncPos = motorEncData * 360 / (64 * 26.9 * 4);
  hardwareInterface.spinOnce();

  // arm position
  armEncData = readAS5147P(2);
  armEncPos = armEncData * 360 / 16383;//Correct
  //armEncPos = (armEncPos - calibArmEnc) * (-1) - 7.43;
//  armEncPos = (armEncPos - calibArmEnc) * (-1) + 12;
  armEncPos = (armEncPos - calibArmEnc) * (-1);
  hardwareInterface.spinOnce();

  //motor velocity
  motorEncVel = (motorEncData - motorEncPosOld); /// dt;

  //arm velocity
  armEncVel = (armEncData - armEncPosOld); /// dt;

  // assign to states
  encSignals.data[0] = motorEncPos;
  encSignals.data[1] = armEncPos;
  encSignals.data[2] = motorEncVel;
  encSignals.data[3] = armEncVel;

  // set encoder positions to old
  motorEncPosOld = motorEncData;
  armEncPosOld = armEncData;

  //publish values
  encPub.publish(&encSignals);
}

