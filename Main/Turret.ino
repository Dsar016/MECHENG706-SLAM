#include <Servo.h> 
#include "Turret.h"

Turret::Turret(int scanAngle)
{
    /*pinMode(FAN_PIN, OUTPUT);
    pinMode(IR_PIN1, INPUT);
    pinMode(IR_PIN2, INPUT);
    pinMode(IR_PIN3, INPUT);
    pinMode(IR_PIN4, INPUT);*/
    Serial.begin(115200);
    
    m_MinAngle = 1500 - (600.0/90.0)*scanAngle;
    m_MaxAngle = 1500 + (600.0/90.0)*scanAngle;

    m_turretServo.attach(SERVO_PIN, m_MinAngle, m_MaxAngle); //, m_MinAngle, m_MaxAngle);
    m_turretServo.writeMicroseconds(1500); //reset servo to straight

    m_currentDir = RIGHT;
}

void Turret::Run(int deltaT)
{
    
    /*if(!m_fireDetected){
        m_fireDetected = RunScan();
    }
    else{
        ExtinguishFire();
    }*/
    //delay(20);
    RunScan(deltaT);
}

bool Turret::RunScan(int deltaT)
{
    m_TimeRunning += deltaT;  
    //if(m_TimeRunning <= 0) return; //increase this number to slow down scan speed
    
    //Update Servo Position
    int currentPos = m_turretServo.readMicroseconds();
    /*if (currentPos - 50 < m_MinAngle){
      m_turretServo.writeMicroseconds(m_MaxAngle);
    }
    else{
      m_turretServo.writeMicroseconds(m_MinAngle);
    }
    m_TimeRunning = 0;*/
    //Serial.println(currentPos);
    if (currentPos + m_currentDir < m_MinAngle){
        m_currentDir = LEFT;
    } 
    else if (currentPos + m_currentDir > m_MaxAngle){
        m_currentDir = RIGHT;
    }

    m_turretServo.writeMicroseconds(currentPos + m_currentDir);
    m_TimeRunning = 0;

    //check for fire
    float threshold = 0; //brightness threshold for determining. 
    
    //check each IR diode, if one is above the threshold. Print true

    // if(IRx > threshold){
    //     return true;
    // }
}

bool Turret::ExtinguishFire()
{
    //contstantly position Servo towards brightest spot
    //Turn on fan
    //once brightness threshold is dropped, stop fan, m_fireDetected = false
}