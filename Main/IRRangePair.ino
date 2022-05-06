#include "IRRangePair.h"

IRRangePair::IRRangePair(short IR_PIN_1, short IR_PIN_2, float sep_length) : 
    IR_PIN_1(IR_PIN_1), IR_PIN_2(IR_PIN_2), sep_length(sep_length)
{

}

//updates range sensor readings
void IRRangePair::Run()
{
    curretDist1 = analogRead(IR_PIN_1); // * some calibration function
    curretDist2 = analogRead(IR_PIN_2);
}

float IRRangePair::getAngle()
{
    //get angle between 
    // tan((dist1-dist2)/sep_length)
}

float IRRangePair::getMeanDist()
{
    return (curretDist1 + curretDist2)/2;
}