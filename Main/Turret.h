#include <Servo.h> 

/**
 * Class for handling turret servo, fan, and IR diodes. 
 * Responsible for fire scannning and extinguishing
 */
#ifndef TURRET_H
#define TURRET_H
class Turret
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________

        Turret(int scanAngle);

        void Run(int deltaT);

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short
        SERVO_PIN = 2,
        FAN_PIN = 0, 
        IR_PIN1 = 0, 
        IR_PIN2 = 0,
        IR_PIN3 = 0,
        IR_PIN4 = 0;

        int m_MinAngle = 600; //0deg 
        int m_MaxAngle = 2400; //180deg
        
        Servo m_turretServo;
        enum direct{
            RIGHT = -1,
            LEFT = 1
        };
        direct m_currentDir;
        int m_TimeRunning; //stores the time in ms that the servo has been driving to its target

        bool m_fireDetected = false;
        
        bool RunScan(int deltaT);

        bool ExtinguishFire();

};
#endif