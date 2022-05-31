#include <Servo.h> 
#include "Turret.h"

Turret::Turret(int scanAngle)
{
    pinMode(FAN_PIN, OUTPUT);
    for (int i = 0; i < sizeof(PT_PINS); i++){
        pinMode(PT_PINS[i], INPUT);
    }
    
    m_MinAngle = 1500 - (600.0/90.0)*scanAngle;
    m_MaxAngle = 1500 + (600.0/90.0)*scanAngle;

    m_turretServo.attach(SERVO_PIN, m_MinAngle, m_MaxAngle); //, m_MinAngle, m_MaxAngle);
    m_turretServo.writeMicroseconds(1500); //reset servo to straight

    m_currentDir = RIGHT;
}

void Turret::Run(int deltaT)
{
    UpdatePTState();            
    SetFan(m_fireDetected);   
    if(m_fireDetected){
        ExtinguishFire();
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

    m_turretServo.writeMicroseconds(currentPos + m_currentDir);
    m_TimeRunning = 0;
}

/**
 * @brief sets turret rotate direction towards brightest light source 
 */
bool Turret::ExtinguishFire()
{
    int rightBias = 0, leftBias = 0;
    int n = sizeof(m_currentPTState);

    int i;
    for (i = 0; i < (int)(n/2); i++){
        rightBias += m_currentPTState[i];
    }
    for (i++; i < n-(int)(n/2); i++){
       leftBias += m_currentPTState[i];
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
    for (int i = 0; i < sizeof(m_currentPTState); i++){
        m_currentPTState[i] = 1-digitalRead(PT_PINS[i]);
        if(m_currentPTState[i] == 1){
            m_fireDetected = true;
        }
    }
}

void Turret::SetFan(bool on)
{
    if(digitalRead(FAN_PIN) != on){
        digitalWrite(FAN_PIN, on);
        
//Turn on fan with MOSFET for  seconds
      /* digitalWrite(MOSFETPIN, HIGH);
      delay(3000);
      digitalWrite(MOSFETPIN, LOW);
      delay(3000);*/
      
    }
}
