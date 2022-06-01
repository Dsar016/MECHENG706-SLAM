#ifndef SONARSENSOR_H
#define SONARSENSOR_H
class SonarSensor
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________

        SonarSensor();

        void Run();

        float GetDist() { return currentDist; }

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const int TRIG_PIN = 48;
        const int ECHO_PIN = 49;
        float currentDist = 0;
        int ms_since_trig = 0;

        const unsigned int MAX_DIST = 23200;

        //min 10 microsecond overhead  fix this


};
#endif
