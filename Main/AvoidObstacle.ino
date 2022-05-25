#include "AvoidObstacle.h"
#include <SoftwareSerial.h>
#include "Serial.h"

AvoidObstacle::AvoidObstacle() {this->right = 0;this->back = 0;}

// Find the fuzzy logic values of Near for input sensor
double AvoidObstacle::Near(double distance) {
  if (distance <= minDist) { return 1; }
  else if (distance >= maxDist) {return 0;}
  else { return ((-distance + maxDist) / (maxDist - minDist)); }
}

// Find the fuzzy logic values of Far for input sensor
double AvoidObstacle::Far(double distance) {
  if (distance <= minDist) { return 0; }
  else if (distance >= maxDist) { return 1; }
  else { return (distance/(maxDist - minDist))-(minDist/(maxDist - minDist)); }
}

// Fuzzification of input sensor distances
void AvoidObstacle::Fuzzify(double LeftIR, double LeftFIR, double Sonar, double RightFIR, double RightIR) {
  double LF[2] = { Near(LeftFIR), Far(LeftFIR) };
  double RF[2] = { Near(RightFIR), Far(RightFIR) };
  double S[2] = { Near(Sonar), Far(Sonar) };
  double R[2] = { Near(RightIR), Far(RightIR) };
  double L[2] = { Near(LeftIR), Far(LeftIR) };

  // Inference
  double Right, Left;
  //Left = (L[1] + LF[1] + RF[0] + R[0])/4;
  //Right = (R[1] + RF[1] + LF[0] + L[0])/4;
  //this->back = (LF[0] + S[0] + RF[0])/3;

  Left = (LF[1] + RF[0])/2;
  Right = (RF[1] + LF[0])/2;
  this->back = (LF[0] + S[0] + RF[0])/3;
  
  this->right = Right-Left;
  return;
}
