//file includes
#include "Chassis.h"
#include "Turret.h"
#include "Gyro.h"
#include "SonarSensor.h"
#include "IRRangePair.h"
#include "Battery.h"
#include "Serial.h"
#include "AvoidObstacle.h"

enum STATE {
  INITIALISING,
  SCAN360,
  DRIVING,
  WALLTURN,
  STOPPING, 
};

STATE state, prevState;
float deltaT = 1; //Running Period
int msCounter0 = 0;
int constant0 = 0;

Chassis* chassis;
Turret* turret;
Gyro* gyro;
SonarSensor* sonarSensor;
IRRangePair* LeftRangePair;
IRRangePair* RightRangePair;
Battery* battery;
AvoidObstacle* avoidobstacle;

void setup()
{
  state = INITIALISING;
  prevState = INITIALISING;

  chassis = new Chassis();
  turret = new Turret(110);
  gyro = new Gyro();
  sonarSensor = new SonarSensor();
  LeftRangePair = new IRRangePair(A12, A13, 10);
  RightRangePair = new IRRangePair(A15, A14, 10);
  battery = new Battery();
  avoidobstacle = new AvoidObstacle();

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  Serial.begin(115200);
  Serial.println("ROBUSSY ROLLOUT");
  delay(2000);
}

void loop()
{
  prevState = state;
  
  switch(state)
  {
    case INITIALISING :   Initialising(deltaT);       break;
    case SCAN360 :        Scan360(deltaT);            break;
    case DRIVING :        Driving(deltaT);            break;
    case WALLTURN :       WallTurn(deltaT);           break;
    case STOPPING :       Stopping(deltaT);           break;
  } 
  if(state != prevState){
    msCounter0 = 0;
    constant0 = 0;
  } //zero counters on state change
  
  delay(deltaT);
}

void Initialising(float deltaT)
{
  // Check Battery
  battery->Check();
  delay(2000);
  
  // Begin tasks
  state = SCAN360;
}

void Scan360(float deltaT)
{ 
  if(constant0 == 0){constant0 = random(-1000, 1000);}
  if(msCounter0 > 3000+constant0){ //scan for random amount of seconds
    state=DRIVING;
  }
  msCounter0 += deltaT;

  turret->servoSpeed = 0;
  turret->Run(deltaT);
  
  chassis->SetSpeed(0, 0, 50);
  if(turret->m_fireDetected){
    chassis->SetSpeed(0, 0, 0);
    state = DRIVING;
    //Serial.println("DRIVING");
  }
  chassis->Run(deltaT);
}

void Driving(float deltaT)
{
    if(msCounter0 > 1000 && !turret->m_fireDetected){ //only scan for 5 seconds
      state=SCAN360;
    }
    msCounter0 += deltaT;
    
    // Update Sensors
    LeftRangePair->Run();
    RightRangePair->Run();
    sonarSensor->Run();

    turret->servoSpeed = 10;

    // Collision manager
    avoidobstacle->Fuzzify(LeftRangePair->getDist1(), LeftRangePair->getDist2(), sonarSensor->GetDist(), RightRangePair->getDist1(), RightRangePair->getDist2());

    // Handle Fire Tracking
    turret->Run(deltaT);

    // Update Speeds
    if(turret->m_fireReached){
      turret->servoSpeed = 1;
      chassis->SetSpeed(0, 0, 0);
    }
    else{chassis->SetSpeed(3*(2 - 4 * avoidobstacle->back),8*avoidobstacle->right, 30*turret->GetFireDirection());}
    
    chassis->Run(deltaT);
}

void WallTurn(float deltaT)
{
  // Update Sensors
  LeftRangePair->Run();
  RightRangePair->Run();
  sonarSensor->Run();

  chassis->SetSpeed(0, 0, 30);
  if(LeftRangePair->getDist1() < 10 && RightRangePair->getDist1() < 10 && sonarSensor->GetDist() < 10){
      state = DRIVING;
  }
  chassis->Run(deltaT);
}

void Stopping(float deltaT)
{
  chassis->StopMotors();
}
