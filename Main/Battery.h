/**
 * Class for handling battery checks
 */
#ifndef BATTERY_H
#define BATTERY_H
class Battery
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________
        Battery();
        void Check();

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________
        bool isOk();
};
#endif