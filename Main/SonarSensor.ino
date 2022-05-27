#include "SonarSensor.h"

SonarSensor::SonarSensor()
{
  
}

//min 10 micro overhead
void SonarSensor::Run()
{
    UpdateDist();
    Serial.println(digitalRead(TRIG_PIN));
}

void SonarSensor::UpdateDist()
{
    unsigned long t1, t2;
    unsigned long pulse_width;

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for pulse on echo pin
    t1 = micros();
    while (digitalRead(ECHO_PIN) == 0);

    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST)) currentDist = static_cast<float>(MAX_DIST);

    // Calculate distance in centimeters. The constants
    // are found in the datasheet, and calculated from the assumed speed
    //of sound in air at sea level (~340 m/s).
    currentDist = pulse_width / 58.0; //cm
}
