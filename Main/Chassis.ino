#include <Servo.h> 
#include "Chassis.h"
#include <SoftwareSerial.h>
HardwareSerial *SerialCom;


/**
 * Class for handling driving robot, odometry and battery checks
 */
Chassis::Chassis()
{
    SerialCom = &Serial;
    SerialCom->begin(115200);
    EnableMotors();
}

//main chassis function, called each loop
void Chassis::Run(float deltaT)
{
    UpdateSpeeds();
    UpdateOdometry(deltaT);
    /*if(!is_battery_voltage_OK)
    {
        StopMotors();
        return;
    }*/    
}

/* Updates the robots speed in each degree of freedom */
void Chassis::SetSpeed(int x_vel, int y_vel, int z_vel)
{
    this->x_vel = (x_vel != NULL) ? x_vel : this->x_vel;
    this->y_vel = (y_vel != NULL) ? y_vel : this->y_vel;
    this->z_vel = (z_vel != NULL) ? z_vel : this->z_vel;
}

void Chassis::ResetOdometry()
{
    x_pos = 0;
    y_pos = 0;
    z_pos = 0;
}



void Chassis::UpdateOdometry(float deltaT)
{
    x_pos += x_vel * deltaT; // * some constant
    y_pos += y_vel * deltaT; // * some constant
    z_pos += z_vel * deltaT; // * some constant
}

/* Configures motor PWM signals to current speed */
void Chassis::UpdateSpeeds()
{
    left_front_motor.writeMicroseconds(1500 + x_vel + y_vel + (L1+L2)*z_vel);
    left_rear_motor.writeMicroseconds(1500 + x_vel - y_vel + (L1+L2)*z_vel);
    right_rear_motor.writeMicroseconds(1500 - x_vel - y_vel  + (L1+L2)*z_vel);
    right_front_motor.writeMicroseconds(1500 - x_vel + y_vel + (L1+L2)*z_vel);
}

/* detaches all motors from GPIO pins. Sets GPIO pinmodes to input */
void Chassis::DisableMotors()
{
    left_front_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
    left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
    right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
    right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

    pinMode(left_front, INPUT);
    pinMode(left_rear, INPUT);
    pinMode(right_rear, INPUT);
    pinMode(right_front, INPUT);
}

/* reattaches all motors to GPIO pins. Sets GPIO pinmodes to input */
void Chassis::EnableMotors()
{
    SerialCom->println("enabling");
    left_front_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
    left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
    right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
    right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On

    pinMode(left_front, OUTPUT);
    pinMode(left_rear, OUTPUT);
    pinMode(right_rear, OUTPUT);
    pinMode(right_front, OUTPUT);
}

void Chassis::StopMotors()
{
    left_front_motor.writeMicroseconds(1500);
    left_rear_motor.writeMicroseconds(1500);
    right_rear_motor.writeMicroseconds(1500);
    right_front_motor.writeMicroseconds(1500);
}
