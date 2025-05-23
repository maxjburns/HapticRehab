#include <Servo.h>

// DONT FORGET TO SET THESE UP. Order is important, and will be outlined formally once we have pinouts
const uint8_t SERVO_PIN = 0;
const uint8_t ECC_PINS[] = {6, 2, 9, 4, 8, 3};

const uint8_t START_BYTE = 254;
const uint8_t END_BYTE = 255;

// Tried 50, too light, tried 100, too high (at 5v)
// DO NOT INCREASE ABOVE 253! we reserve 254 and 255 for start and end bytes
const uint8_t motorMax = 253;

const uint8_t totalECCPins = sizeof(ECC_PINS) / sizeof(ECC_PINS[0]);
// servo, ECC pins
const uint8_t maxCommandSize = 1 + 1 + totalECCPins;

uint8_t commandList[maxCommandSize];
Servo feedbackServo;
uint8_t commandIndex = 0;

void setup() {
  
  for (uint8_t i = 0; i < totalECCPins; i++) 
  {
    pinMode(ECC_PINS[i], OUTPUT);
  }
  
  feedbackServo.attach(SERVO_PIN);

  Serial.begin(9600);
}

void loop() {
  // Serial messages should come in the following format:
  // "\nm......,"
  // m --> mode {"s" --> servo only
  //             "m" --> motor only}
  // motors are driven and stopped, one after the other.
  /*driveMotor(1, motorMax);
  delay(delay1);
  stopMotor(1);

  driveMotor(2, motorMax);
  delay(delay2);
  stopMotor(2);

  driveMotor(3, motorMax);
  delay(delay3);
  stopMotor(3);
  delay(endDelay);*/
  if(Serial.available())
  {
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
    // command ongoing, add to list in correct index
    else
    {
      commandList[commandIndex] = newByte;
      commandIndex++;
    }
  }
 
}

void sendCommands(uint8_t commandList[])
{
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
  if(motorIndex >= totalECCPins)
  {
    Serial.println("ERROR: INCORRECT MOTOR INDEX!");
  }
  // used to drive motor based on index
  uint8_t pin = ECC_PINS[motorIndex];
  uint8_t actualDutyCycle;

  // Limit duty cycle
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
  // convenient to stop motor
  driveMotor(motorIndex, 0);
}