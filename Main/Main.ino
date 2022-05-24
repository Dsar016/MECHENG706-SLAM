//file includes
#include "Chassis.h"
#include "Turret.h"
#include "Gyro.h"
#include "SonarSensor.h"
#include "IRRangePair.h"
#include "Battery.h"
#include "Serial.h"
#include "AvoidObstacle.h"

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
IRRangePair* FrontRangePair;
IRRangePair* RightRangePair;
Battery* battery;
AvoidObstacle* avoidobstacle;

void setup()
{
  state = INITIALISING;

  chassis = new Chassis();
  turret = new Turret(45);
  gyro = new Gyro();
  sonarSensor = new SonarSensor();
  FrontRangePair = new IRRangePair(A13, A12, 10); // fix these vals
  RightRangePair = new IRRangePair(A15, A14, 10);
  battery = new Battery();
  avoidobstacle = new AvoidObstacle();

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
    // Update Sensors
    FrontRangePair->Run();
    RightRangePair->Run();
    sonarSensor->Run();

    // Collision manager
    avoidobstacle->Fuzzify(FrontRangePair->getDist1(), FrontRangePair->getDist2(), sonarSensor->GetDist(), RightRangePair->getDist1(), RightRangePair->getDist2());

    chassis->SetSpeed(100-avoidobstacle->back, 50*avoidobstacle->right, 0);
    chassis->Run(10);
}

void Blowing(float deltaT)
{

}

void Stopping(float deltaT)
{
  //disable_motors();
}
