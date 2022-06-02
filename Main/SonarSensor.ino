#include "SonarSensor.h"

SonarSensor::SonarSensor()
{
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void SonarSensor::Run()
{
  if(ms_since_trig < 10){
    ms_since_trig++;
    return;
  }
  ms_since_trig = 0;
  
  unsigned long t1, t2, pulse_width;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  while (digitalRead(ECHO_PIN) == 0 ) {}
  
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1){}
  t2 = micros();
  
  pulse_width = t2 - t1;

  prevDist = currentDist;
  currentDist = pulse_width / 58.0;

  //add smoothing to ultrasonic reading
  if(prevDist*10 < currentDist){
      currentDist = prevDist*10;
  }

  
}
