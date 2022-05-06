#ifndef GYRO_H
#define GYRO_H
class Gyro
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________

        float currentPos = 0;
        float currentRate = 0;
        float prevRate = 0;

        Gyro();

        //updates gyro readings
        void Run(float deltaT);

        void Calibrate();

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short GYRO_PIN = A3;
        const float supplyVoltage = 5.0f;  
        float zeroVoltage = 0.0f;   
        const float sensitivity = 0.0065f; // 0.007 from datasheet      
        float rotationThreshold = 1.5;  
};
#endif