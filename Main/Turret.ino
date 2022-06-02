#include <Servo.h> 
#include "Turret.h"

Turret::Turret(int scanAngle)
{
    pinMode(FAN_PIN, OUTPUT);
    
    m_MinAngle = m_StraightAngle - (600.0/90.0)*scanAngle;
    m_MaxAngle = m_StraightAngle + (600.0/90.0)*scanAngle;

    m_turretServo.attach(SERVO_PIN, m_MinAngle, m_MaxAngle); //, m_MinAngle, m_MaxAngle);
    m_turretServo.writeMicroseconds(m_StraightAngle); //reset servo to straight

    m_currentDir = RIGHT;

    digitalWrite(FAN_PIN, LOW);

}

void Turret::Run(int deltaT)
{
    UpdatePTState(); 
    //ExtinguishFire();
    //Serial.println(m_fireDetected);   
    GetFireDirection();       
    SetFan(m_fireDetected);   
    if(m_fireDetected){
         ExtinguishFire();
    }
    else if(m_currentDir == STRAIGHT){
        m_currentDir = RIGHT;
    }
    
    RunScan(deltaT);
}

/**
 * @brief updates servo target position based on rotation direction and min/max rotate angles
 */
bool Turret::RunScan(int deltaT)
{
    m_TimeRunning += deltaT;  
    if(m_TimeRunning <= 0) return; //increase this number to slow down response speed
    
    int currentPos = m_turretServo.readMicroseconds();

    if (currentPos + m_currentDir < m_MinAngle){
        m_currentDir = LEFT;
    } 
    else if (currentPos + m_currentDir > m_MaxAngle){
        m_currentDir = RIGHT;
    }
    m_turretServo.writeMicroseconds(currentPos + 5*m_currentDir);
    m_TimeRunning = 0;
}

/**
 * @brief sets turret rotate direction towards brightest light source 
 */
bool Turret::ExtinguishFire()
{
    int rightBias = 0, leftBias = 0;

    int i = 0;
    for (i; i < (int)(PT_NUM/2); i++){
        leftBias += m_currentPTState[i];
    }
    for (i; i < PT_NUM; i++){
       rightBias += m_currentPTState[i];
    }

    if(rightBias == leftBias)       {m_currentDir = STRAIGHT;}
    else if(rightBias > leftBias)   { m_currentDir = RIGHT;}
    else if(rightBias < leftBias)   {m_currentDir = LEFT;}
}

/**
 * @brief checks all phototransistors and writes their states to an array.
 * if any states are high, raises fire-detected flag
 */
bool Turret::UpdatePTState()
{
    m_fireDetected = false;
    for (int i = 0; i < PT_NUM; i++){
        m_currentPTState[i] = 1-digitalRead(PT_PINS[i]);
        if(m_currentPTState[i] == 1){
            m_fireDetected = true;
        }
    }
}

int Turret::GetFireDirection(){
  if(!m_fireDetected){
    //return STRAIGHT
    //Serial.println("STRAIGHT");
    return STRAIGHT;
  }
  
  int straightThreshold = 15;
  int straightMin = m_StraightAngle - (600.0/90.0)*straightThreshold;
  int straightMax = m_StraightAngle + (600.0/90.0)*straightThreshold;
  /*Serial.print(straightMin);
  Serial.print(" ");
  Serial.print(straightMax);
  Serial.print(" ");*/

  int currentPos = m_turretServo.readMicroseconds();
  /*Serial.print(currentPos);
  Serial.print(" ");*/


  if(currentPos > straightMin && currentPos < straightMax){/*Serial.println("STRAIGHT");*/ return STRAIGHT;}
  else if(currentPos < 1500){/*Serial.println("RIGHT");*/ return RIGHT;}
  else if(currentPos > 1500){/*Serial.println("LEFT");*/ return LEFT;}
}

void Turret::SetFan(bool on)
{
  if(on){digitalWrite(FAN_PIN, HIGH);}
  else{digitalWrite(FAN_PIN, LOW);}       
}
