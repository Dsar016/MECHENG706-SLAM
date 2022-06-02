#include <Servo.h> 
#include "Chassis.h"
#include <SoftwareSerial.h>

Chassis::Chassis()
{
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
    ThetaOne = one_over_Rw * (x_vel + y_vel - (L + t) * z_vel);
    ThetaTwo = one_over_Rw * (x_vel - y_vel + (L + t) *z_vel);
    ThetaThree = one_over_Rw * (x_vel - y_vel - (L + t) *z_vel);
    ThetaFour = one_over_Rw * (x_vel + y_vel + (L + t) *z_vel);
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
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour); 
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
    Serial.println("enabling");
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
