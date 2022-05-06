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

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________

        const short IR_PIN_1;
        const short IR_PIN_2;
        const float sep_length; //separation distance between IR range sensors

        float curretDist1 = 0;
        float curretDist2 = 0;
};
#endif