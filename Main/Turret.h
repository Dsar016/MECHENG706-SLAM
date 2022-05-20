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

        bool m_fireDetected = false;

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short SERVO_PIN = 21;
        const short FAN_PIN = 0;

        const short PT_PINS[4] = {0, 0, 0, 0};
        int m_currentPTState[4] = {1,1,1,1};

        int m_MinAngle = 600; //0deg 
        int m_MaxAngle = 2400; //180deg
        
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
