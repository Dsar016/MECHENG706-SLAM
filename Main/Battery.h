/**
 * Class for handling battery checks
 */
#ifndef BATTERY_H
#define BATTERY_H
class Battery
{
    public: //PUBLIC_MEMBERS_______________________________________________________________________________________
        Battery(HardwareSerial &Serial);
        void Check();
        HardwareSerial *SerialCom;

    private: //PRIVATE_MEMBERS_______________________________________________________________________________________
        bool isOk();
};
#endif