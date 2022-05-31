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

        float getDist1();

        float getDist2();

        double KalmanFilter(double rawdata, double prev_est);

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short IR_PIN_1;
        const short IR_PIN_2;
        const float sep_length; //separation distance between IR range sensors

        const double A12_Exponent = -0.992;
        const double A12_Value = 2256.1;
        const double A13_Exponent = -1.261;
        const double A13_Value = 22388;
        const double A14_Exponent = -0.963;
        const double A14_Value = 1933.2;
        const double A15_Exponent = -1.191;
        const double A15_Value = 15714;

        float currentDist1 = 0;
        float currentDist2 = 0;
        
        //Kalman Filter values
        double last_est_1 = 15;

        double last_est_2 = 15;

        //Might make this diffent for each Kalman Filter
        double last_var = 999;
        
        double process_noise = 10; //High if the process itself has lots of noise
        double sensor_noise = 1; //High if the sensor has lots of noise
        //Note: these noises are relative to each other, so if the process is stable, the sensor noise value will be larger due to this
};
#endif
