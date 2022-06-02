#include "SonarSensor.h"

SonarSensor::SonarSensor()
{
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void SonarSensor::Run()
{
  if(ms_since_trig < 50){
    ms_since_trig++;
    return;
  }
  ms_since_trig = 0;

  prevDist = currentDist;
  currentDist = Pulse();

  //add smoothing to ultrasonic reading
  if(prevDist+30 < currentDist){
      currentDist = prevDist+30;
  }
}

float SonarSensor::Pulse()
{
  unsigned long t1, t2, pulse_width;
  float maxPulse = 58.0*maxDist;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  t1 = micros(); t2 = micros();
  while (digitalRead(ECHO_PIN) == 0){
    t2 = micros();
    if(t2 - t1 >= maxPulse){return maxDist;}
  }
  
  t1 = micros(); t2 = micros();
  while (digitalRead(ECHO_PIN) == 1){
    t2 = micros();
    if(t2 - t1 >= maxPulse){return maxDist;}
  }
  
  return (t2 - t1) / 58.0;
}
