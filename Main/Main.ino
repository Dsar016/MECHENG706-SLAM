//file includes
#include "Chassis.h"
#include "Turret.h"
#include "SonarSensor.h"
#include "IRRangePair.h"
#include "Battery.h"
#include "AvoidObstacle.h"

enum STATE {
  INITIALISING,
  SCAN360,
  DRIVING,
  STOPPING, 
};

STATE state, prevState;
float deltaT = 1; //Running Period
int msCounter0 = 0;
int constant0 = 0;
int firesExtinguished = 0;
bool wasHigh = false;

Chassis* chassis;
Turret* turret;
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
  sonarSensor = new SonarSensor();
  LeftRangePair = new IRRangePair(A12, A13, 10);
  RightRangePair = new IRRangePair(A15, A14, 10);
  battery = new Battery();
  avoidobstacle = new AvoidObstacle();

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  Serial.begin(115200);
  Serial.println("SHARKY ROLLOUT");
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
  delay(1000);
  
  // Begin tasks
  state = SCAN360;
}

void Scan360(float deltaT)
{ 
  // Scan for random amount of seconds
  if(constant0 == 0){constant0 = random(-1000, 1000);}
  if(msCounter0 > 3000+constant0){
    state=DRIVING;
  }
  msCounter0 += deltaT;

  // Handle turret
  turret->Straighten();
  turret->servoSpeed = 0;
  turret->Run(deltaT);

  // Handle movement
  chassis->SetSpeed(0, 0, 50);
  if(turret->m_fireDetected){
    state = DRIVING;
  }
  chassis->Run(deltaT);
}

void Driving(float deltaT)
{
    // Execute scan after certain driving time
    if(msCounter0 > 750 && !turret->m_fireDetected){
      state=SCAN360;
    }
    msCounter0 += deltaT;
    
    // Update Sensors
    LeftRangePair->Run();
    RightRangePair->Run();
    sonarSensor->Run();

    // Collision manager
    avoidobstacle->Fuzzify(LeftRangePair->getDist1(), LeftRangePair->getDist2(), sonarSensor->GetDist(), RightRangePair->getDist1(), RightRangePair->getDist2());

    // Handle Fire Tracking
    turret->servoSpeed = 12;
    turret->Run(deltaT);

    if(turret->m_fireReached){
      wasHigh = true;
    }

    if(wasHigh){
      if(!turret->m_fireDetected){
        firesExtinguished++;
        wasHigh = false;
      }
    }

    if(firesExtinguished >= 2){
      state=STOPPING;
    }

    // Update Speeds
    if(wasHigh){//turret->m_fireReached){
      turret->servoSpeed = 5;
      chassis->SetSpeed(0, 0, 0);
    }
    else{chassis->SetSpeed(3*(2 - 4 * avoidobstacle->back),12*avoidobstacle->right, 30*turret->GetFireDirection());}
    
    chassis->Run(deltaT);
}

void Stopping(int deltaT)
{
  Serial.println("STOPPING");
  chassis->SetSpeed(0, 0, 0);
  chassis->Run(deltaT);
}
