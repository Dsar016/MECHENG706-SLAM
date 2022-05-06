// Wireless Serial ////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
///////////////////////////////////////////////////////////////////////////

//file includes
#include "Chassis.h"
#include "Turret.h"
#include "Gyro.h"
#include "SonarSensor.h"
#include "IRRangePair.h"

enum STATE {
  INITIALISING,
  DRIVING,
  BLOWING,
  STOPPING, 
  TA_ELIMINATE
};

STATE state;
float deltaT = 1; //Running Period
//HardwareSerial *SerialCom;

Chassis* chassis;
Turret* turret;
Gyro* gyro;
SonarSensor* sonarSensor;
IRRangePair* sideRangePair;
IRRangePair* forwardRangePair;

void setup()
{
  state = DRIVING;

  chassis = new Chassis();
  turret = new Turret();
  gyro = new Gyro();
  sonarSensor = new SonarSensor();
  sideRangePair = new IRRangePair(A15, A13, 10); // fix these vals
  forwardRangePair = new IRRangePair(A14, A11, 10);

  //Serial Pointer
  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  /*SerialCom = &Serial;
  SerialCom->begin(115200);*/
  BluetoothSerial.begin(115200);
  //SerialCom->println("ROBUSSY ROLLOUT");
}

void loop()
{
  switch(state)
  {
    case INITIALISING :   Initialising(deltaT);       break;
    case DRIVING :        Driving(deltaT);            break;
    case BLOWING :        Blowing(deltaT);            break;
    case STOPPING :       Stopping(deltaT);           break;
  } 
  delay(deltaT);
}

void Initialising(float deltaT)
{

}

void Driving(float deltaT)
{
  chassis->SetSpeed(100, 0, 0);
  chassis->Run(deltaT);
  sonarSensor->Run();

  if(sonarSensor->GetDist() < 100){
    state = BLOWING;
  }
  // Do driving things        ex) Motor.SetSpeed(x_speed, y_speed, z_speed)

  // Change state if necessary:
  // if(turret.detectlight()){
  //   state = BLOWING;
  //   return;
  // }
}

void Blowing(float deltaT)
{

}

void Stopping(float deltaT)
{
  //disable_motors();
}
