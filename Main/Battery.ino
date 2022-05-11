#include "Battery.h"
#include <SoftwareSerial.h>
#include "Serial.h"

Battery::Battery() {}

// Check the battery voltage is stable during initialisation
void Battery::Check()
{
    int Count = 0;

    while(Count < 10) {
        // Check for stable battery
        if(isOk()) {
            Count += 1;
        } else {
            Count = 0;
        }
        
    }
    
    Serial.println("Battery Ok");
}

bool Battery::isOk() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Serial.print("Lipo level:");
    Serial.print(Lipo_level_cal);
    Serial.print("%");
    // Serial.print(" : Raw Lipo:");
    // Serial.println(raw_lipo);
    Serial.println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      Serial.println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      Serial.println("!Lipo is Overchanged!!!");
    else {
      Serial.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      Serial.print("Please Re-charge Lipo:");
      Serial.print(Lipo_level_cal);
      Serial.println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}