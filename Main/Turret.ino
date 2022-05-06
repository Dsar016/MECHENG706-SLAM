#include <Servo.h> 
#include "Turret.h"

Turret::Turret()
{
    pinMode(FAN_PIN, OUTPUT);
    pinMode(IR_PIN1, INPUT);
    pinMode(IR_PIN2, INPUT);
    pinMode(IR_PIN3, INPUT);
    pinMode(IR_PIN4, INPUT);

    m_turretServo.attach(SERVO_PIN);
    m_turretServo.writeMicroseconds(90); //reset servo to straight

    m_currentDir = RIGHT;
}

void Turret::Run()
{
    if(!m_fireDetected){
        m_fireDetected = RunScan();
    }
    else{
        ExtinguishFire();
    }
}

bool Turret::RunScan()
{
    //Update Servo Position
    int currentPos = m_turretServo.readMicroseconds();
    if ((currentPos + m_currentDir < 0) && (m_currentDir == RIGHT)){
        m_currentDir = LEFT;
    } 
    if ((currentPos + m_currentDir > 180) && (m_currentDir == LEFT)){
        m_currentDir = RIGHT;
    }
    m_turretServo.writeMicroseconds(currentPos + m_currentDir);

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
