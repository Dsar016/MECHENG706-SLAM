#include "Gyro.h"

Gyro::Gyro()
{
    Calibrate();
}

//updates gyro readings
void Gyro::Run(float deltaT)
{
    currentPos += currentRate*=deltaT;
    prevRate = currentRate;
    
    float gyroVal = (analogRead(GYRO_PIN)/1023.0)*supplyVoltage - zeroVoltage;
    currentRate = gyroVal/sensitivity;

    currentRate = abs(currentRate)>rotationThreshold ? currentRate : 0;
}

void Gyro::Calibrate()
{
    float sum = 0;
    int n = 100; //number of measurements 

    for(int i = 0; i < n; i++){
        sum += supplyVoltage*(analogRead(GYRO_PIN)/1023.0);
    }

    zeroVoltage = sum/n; 
}