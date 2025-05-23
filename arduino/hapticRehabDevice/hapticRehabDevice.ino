#include <Servo.h>
// This code is intended to control the haptic rehab device, recieving messages as fast as possible.
// commands are sent as command messages, which are lists of bytes led by a start byte, and closed
// by an end byte. Between these two bytes the control mode is set, and all of the motor and servo values.

// command messages follow this convention:
//  0 | 254 --> start of a message
//  1 | 0x?? --> mode of control (0 --> only servo; 1--> only vibration; 2--> both feedback)
//  2 | 0x?? --> servo command (0 to 180)
//  3 | 0x?? --> motor index 0 command (0 to MOTOR_MAX) (should not exceed 253)
//  4 | 0x?? --> motor index 1 command (0 to MOTOR_MAX) (should not exceed 253)
//  ..
//  n | 255 --> end of a message
//
// Where n is equal to maxCommandSize-1


// DONT FORGET TO SET THESE UP. Order is important, and will be outlined formally once we have pinouts
const uint8_t SERVO_PIN = 0;
const uint8_t ECC_PINS[] = {6, 2, 9, 4, 8, 3};

const uint8_t START_BYTE = 254;
const uint8_t END_BYTE = 255;

// Tried 50, too light, tried 100, too high (at 5v)
// DO NOT INCREASE ABOVE 253! we reserve 254 and 255 for start and end bytes
const uint8_t motorMax = 253;

const uint8_t totalECCPins = sizeof(ECC_PINS) / sizeof(ECC_PINS[0]);
const uint8_t maxCommandSize = 1 + 1 + totalECCPins;

uint8_t commandList[maxCommandSize];
Servo feedbackServo;
uint8_t commandIndex = 0;

void setup() {
  
  // vibration motor setup
  for (uint8_t i = 0; i < totalECCPins; i++) 
  {
    pinMode(ECC_PINS[i], OUTPUT);
  }
  
  // servo setup
  feedbackServo.attach(SERVO_PIN);

  // serial setup
  Serial.begin(9600);
}

void loop() {


  if(Serial.available())
  {
    // read serial data if we can
    uint8_t newByte = Serial.read();

    // new command so let's wipe command list
    if(newByte==START_BYTE)
    {
      memset(commandList, 0, maxCommandSize);
      commandIndex = 0;
    }
    // command complete, time to send command list
    else if(newByte==END_BYTE)
    {
      sendCommands(commandList);
    }
    // command still coming in, add byte to list in correct index
    else
    {
      commandList[commandIndex] = newByte;
      commandIndex++;
    }
  }
 
}

void sendCommands(uint8_t commandList[])
{
  // used to send commands to the motors, once commandList is assembled.
  // mode 0 --> only servo
  // mode 1 --> only vibration
  // mode 2 --> both feedback methods.
  // More modes can be supported in the future.

  uint8_t mode = commandList[0];
  
  // SERVO
  if(mode==0 || mode==2)
  {
    feedbackServo.write(commandList[1]);
  }
  
  // VIBRATION
  else if(mode==1 || mode==2)
  {
    uint8_t eccCommand;

    for (uint8_t i = 0; i < totalECCPins; i++) 
    {
      eccCommand = commandList[i+2];
      driveMotor(i, eccCommand);
    }
  }
  else
  {
    Serial.println("ERROR: INVALID MODE");
  }
  
  
}

void driveMotor(uint8_t motorIndex, uint8_t dutyCycle)
{
  // used to drive the eccentric mass motors. motor index reference can be found at beginning of code.

  if(motorIndex >= totalECCPins)
  {
    Serial.println("ERROR: INCORRECT MOTOR INDEX!");
  }
  // used to drive motor based on index
  uint8_t pin = ECC_PINS[motorIndex];
  uint8_t actualDutyCycle;

  // Limit duty cycle if out of range
  if(dutyCycle > motorMax)
  {
    actualDutyCycle=motorMax;
  }
  else if(dutyCycle < 0)
  {
    actualDutyCycle=0;
  }
  else
  {
    actualDutyCycle = dutyCycle;
  }

  analogWrite(pin, actualDutyCycle);
}

void stopMotor(uint8_t motorIndex)
{
  // convenient to stop a motor
  driveMotor(motorIndex, 0);
}