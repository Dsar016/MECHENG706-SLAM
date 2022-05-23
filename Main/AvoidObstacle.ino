#include "AvoidObstacle.h"
#include <SoftwareSerial.h>
#include "Serial.h"

AvoidObstacle::AvoidObstacle() {this->directionModifier = 0;}

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

double AvoidObstacle::Defuzzify(double Left, double Right) {
  return Right - Left;
}

// Fuzzification of input sensor distances
void AvoidObstacle::Fuzzify(double LeftFIR, double RightFIR, double Sonar, double FrontRIR, double BackRIR) {
  double LF[2] = { Near(LeftFIR), Far(LeftFIR)};
  double RF[2] = { Near(RightFIR), Far(RightFIR) };
  double S[2] = { Near(Sonar), Far(Sonar) };
  double FR[2] = { Near(FrontRIR), Far(FrontRIR) };
  double BR[2] = { Near(BackRIR), Far(BackRIR) };
  
  // Inference
  double Right, Left;
  Left = LF[1] * RF[0]; // Rule B
  Right = LF[1] * S[0] * RF[1] * FR[1] * BR[1]; // Rule C
  Left = Left + LF[1] * S[0] * RF[1] * FR[1] * BR[0]; // Rule D
  Left = Left + LF[1] * S[0] * RF[1] * FR[0]; // Rule E
  Right = Right + LF[0] * RF[1]; // Rule F
  Right = Right + LF[0] * RF[0] * FR[1] * BR[1]; // Rule G
  Left = Left + LF[0] * RF[0] * FR[1] * BR[0]; // Rule H
  Left = Left + LF[0] * RF[0] * FR[0]; // Rule I

  this->directionModifier = Defuzzify(Left, Right);
  return;
}
