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

        Turret();

        void Run();

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short
        SERVO_PIN = 0,
        FAN_PIN = 0, 
        IR_PIN1 = 0, 
        IR_PIN2 = 0,
        IR_PIN3 = 0,
        IR_PIN4 = 0;

        Servo m_turretServo;
        enum direct{
            RIGHT = -1,
            LEFT = 1
        };
        direct m_currentDir;

        bool m_fireDetected = false;
        
        bool RunScan();

        bool ExtinguishFire();

};
#endif
