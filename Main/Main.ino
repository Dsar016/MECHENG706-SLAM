//file includes
#include "Chassis.h"
#include "Turret.h"
#include "Gyro.h"
#include "SonarSensor.h"
#include "IRRangePair.h"
#include "Battery.h"
#include "Serial.h"

extern SoftwareSerial* BluetoothSerial = new SoftwareSerial(BLUETOOTH_RX, BLUETOOTH_TX);

enum STATE {
  INITIALISING,
  DRIVING,
  BLOWING,
  STOPPING, 
  TA_ELIMINATE
};

STATE state;
float deltaT = 1; //Running Period

Chassis* chassis;
Turret* turret;
Gyro* gyro;
SonarSensor* sonarSensor;
IRRangePair* sideRangePair;
IRRangePair* forwardRangePair;
Battery* battery;

void setup()
{
  state = INITIALISING;

  chassis = new Chassis();
  turret = new Turret(45);
  gyro = new Gyro();
  sonarSensor = new SonarSensor();
  sideRangePair = new IRRangePair(A15, A13, 10); // fix these vals
  forwardRangePair = new IRRangePair(A14, A11, 10);
  battery = new Battery();

  //Serial Pointer
  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  Serial.begin(115200);
  BluetoothSerial->begin(115200);
  Serial.println("ROBUSSY ROLLOUT");
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
  // Check Battery
  battery->Check();

  // Begin tasks
  state = DRIVING;
}

void Driving(float deltaT)
{
  turret->Run(deltaT);
  /*chassis->SetSpeed(100, 0, 0);
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
  // }*/
}

void Blowing(float deltaT)
{

}

void Stopping(float deltaT)
{
  //disable_motors();
}
