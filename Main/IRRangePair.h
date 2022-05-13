#ifndef IRRANGEPAIR_H
#define IRRANGEPAIR_H
class IRRangePair
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________

        IRRangePair(short IR_PIN_1, short IR_PIN_2, float sep_length);

        //updates range sensor readings
        void Run();

        float getAngle();

        float getMeanDist();

        double KalmanFilter(double rawdata, double prev_est);

        float currentDist1 = 0;
        float currentDist2 = 0;

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short IR_PIN_1;
        const short IR_PIN_2;
        const float sep_length; //separation distance between IR range sensors

        const double A12_Exponent = -1.024;
        const double A12_Value = 2567.6;
        const double A13_Exponent = -1.073;
        const double A13_Value = 7402.8;
        const double A14_Exponent = -0.976;
        const double A14_Value = 1978.5;
        const double A15_Exponent = -1.151;
        const double A15_Value = 12293;
        
        //Kalman Filter values
        double last_est_1 = 0;

        double last_est_2 = 0;

        //Might make this diffent for each Kalman Filter
        double last_var = 999;
        
        double process_noise = 10; //High if the process itself has lots of noise
        double sensor_noise = 1; //High if the sensor has lots of noise
        //Note: these noises are relative to each other, so if the process is stable, the sensor noise value will be larger due to this
};
#endif
