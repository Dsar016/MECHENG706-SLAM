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
        int GetFireDirection();

        bool m_fireDetected = false;

        int servoSpeed = 5; 

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short SERVO_PIN = 22;
        const short FAN_PIN = 34;

        const short PT_NUM = 4;
        const int PT_PINS[4] = {7, 6, 5, 4};
        int m_currentPTState[4] = {1,1,1,1};

        int m_MinAngle = 600; //0deg 
        int m_MaxAngle = 2400; //180deg
        const int m_StraightAngle = 1425;
        
        Servo m_turretServo;
        enum direct{
            RIGHT = -1,
            STRAIGHT = 0,
            LEFT = 1
        };
        direct m_currentDir;
        int m_TimeRunning; //stores the time in ms that the servo has been driving to its target
        
        bool RunScan(int deltaT);
        bool ExtinguishFire();
        float CalculateFireAngle();
        bool UpdatePTState(); //return true if val above threshold
        void SetFan(bool on);
};
#endif
