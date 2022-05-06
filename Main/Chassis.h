#include <Servo.h> 

/**
 * Class for handling driving robot, odometry and battery checks
 */
#ifndef CHASSIS_H
#define CHASSIS_H
class Chassis
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________

        float x_pos = 0;
        float y_pos = 0;
        float z_pos = 0; //angular
        
        /* Constructor */
        Chassis();

        /* main chassis function, called each loop */
        void Run(float deltaT);

        /* Updates the robots speed in each degree of freedom */
        void SetSpeed(int x_vel, int y_vel, int z_vel);

        void ResetOdometry();

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        enum DIRECTION {
            LEFT,
            RIGHT,
            FORWARDS,
            BACKWARDS,
            CCW,
            CW
        };

        const byte left_front = 46;
        const byte left_rear = 47;
        const byte right_rear = 50;
        const byte right_front = 51;

        Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
        Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
        Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
        Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

        const float L1 = 1;
        const float L2 = 1; 

        int x_vel = 0;
        int y_vel = 0;
        int z_vel = 0; //angular

        // Motor Speed Effects
        int ThetaOne;
        int ThetaTwo;
        int ThetaThree;
        int ThetaFour;

        //Robot Dimensions, specific measurements are shown in our notes
        float L = 0.09; //Unit is metres
        float t = 0.09; //Unit is metres
        float one_over_Rw = 1 / 0.022;

        void UpdateOdometry(float deltaT);

        /* Configures motor PWM signals to current speed */
        void UpdateSpeeds();

        /* detaches all motors from GPIO pins. Sets GPIO pinmodes to input */
        void DisableMotors();

        /* reattaches all motors to GPIO pins. Sets GPIO pinmodes to input */
        void EnableMotors();

        void StopMotors();
};
#endif