#include "IRRangePair.h"

IRRangePair::IRRangePair(short IR_PIN_1, short IR_PIN_2, float sep_length) : 
    IR_PIN_1(IR_PIN_1), IR_PIN_2(IR_PIN_2), sep_length(sep_length)
{

}

//updates range sensor readings
void IRRangePair::Run()
{
    int val1 = analogRead(IR_PIN_1);
    int val2 = analogRead(IR_PIN_2);
    double temp1;
    double temp2;
    switch(IR_PIN_1){
      case A12:      
       temp1 = A12_Value * pow(val1,A12_Exponent);
       currentDist1 = KalmanFilter(temp1, last_est_1);
       last_est_1 = currentDist1;
      case A13:
       temp1 = A13_Value * pow(val1,A13_Exponent);
       currentDist1 = KalmanFilter(temp1, last_est_1);
       last_est_1 = currentDist1;       
      case A14:
       temp1 = A14_Value * pow(val1,A14_Exponent);
       currentDist1 = KalmanFilter(temp1, last_est_1);
       last_est_1 = currentDist1;
      case A15:
       temp1 = A15_Value * pow(val1,A15_Exponent);
       currentDist1 = KalmanFilter(temp1, last_est_1);
       last_est_1 = currentDist1;  
    }

    switch(IR_PIN_2){
      case A12:      
       temp2 = A12_Value * pow(val2,A12_Exponent);
       currentDist2 = KalmanFilter(temp2, last_est_2);
       last_est_2 = currentDist2;
      case A13:
       temp2 = A13_Value * pow(val2,A13_Exponent);
       currentDist2 = KalmanFilter(temp2, last_est_2);
       last_est_2 = currentDist2;
      case A14:
       temp2 = A14_Value * pow(val2,A14_Exponent);
       currentDist2 = KalmanFilter(temp2, last_est_2);
       last_est_2 = currentDist2;
      case A15:
       temp2 = A15_Value * pow(val2,A15_Exponent);
       currentDist2 = KalmanFilter(temp2, last_est_2);
       last_est_2 = currentDist2;       
    }
}

float IRRangePair::getAngle()
{
    //get angle between 
    // tan((dist1-dist2)/sep_length)
}

float IRRangePair::getMeanDist()
{
    return (currentDist1 + currentDist2)/2;
}

double IRRangePair::KalmanFilter(double rawdata, double last_est){
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = last_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}
