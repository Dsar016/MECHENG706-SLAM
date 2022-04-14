/*
  MechEng 706 Base Code
  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project
  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html
  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output


// Wireless Serial ////////////////////////////////////////////////////////
// To print to wireless module use BluetoothSerial.print(...);
#include <SoftwareSerial.h>
// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
///////////////////////////////////////////////////////////////////////////

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HCSR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_READ_IR //Uncomment if IR Sensors not attached
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Robot direction
enum DIRECTION {
  LEFT,
  RIGHT,
  FORWARDS,
  BACKWARDS,
  CCW,
  CW
};

// Moving Average
double CurrentSensor[5]; //IR 1-4 //Ultrasonic //Gyro
double Average[5];
double PrevSensor[5][10];
int timer2i = 0;

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

//IR Range Sensor Pins
const int MID_RANGE_RIGHT_PIN = A14; 
const int MID_RANGE_LEFT_PIN = A11;
const int LONG_RANGE_RIGHT_PIN = A15;
const int LONG_RANGE_LEFT_PIN = A13;

//List of IR range sensors on the robot
enum IR_SENSOR {
  LEFT_MID, 
  LEFT_LONG, 
  RIGHT_MID,
  RIGHT_LONG
};

//Define constants for IR sensor Calculations, excel sheet with calibrations are available on our google drive
const double Mid_Left_Exponent = -1.024;
const double Mid_Left_Value = 2567.6;
const double Long_Left_Exponent = -1.073;
const double Long_Left_Value = 7402.8;
const double Mid_Right_Exponent = -0.976;
const double Mid_Right_Value = 1978.5;
const double Long_Right_Exponent = -1.151;
const double Long_Right_Value = 12293;

double last_est = 15;
double last_var = 999;
double process_noise = 10; //High if the process itself has lots of noise
double sensor_noise = 1; //High if the sensor has lots of noise
//Note: these noises are relative to each other, so if the process is stable, the sensor noise value will be larger due to this

//Map array, which is initially set to 0,0. The 0,0 point will be taken after the robot has found the corner. it will be a nx2 array. 
const int ROW_MAX = 500;
const int COL_MAX = 2;
float Map[ROW_MAX][COL_MAX]; //First column is x position, second column is y position. X position will be along the long end of the table
int MapRowCounter = 0;
int PastMapRowCounter = 0;
float IRReadingSLAM, PastIRReadingSLAM, UltrasonicSLAM, PastUltrasonicSLAM;
DIRECTION PastDirect;
DIRECTION SLAMdirect = LEFT;

//Gyro Analog Pin
const int GYRO_PIN = A3;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29


int speed_val = 100;
int speed_change;
int j = 0;
//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  cli();
  
  sei();

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  BluetoothSerial.begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");
  delay(1000); //settling time but no really needed
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  }
}

///////////////////// PROTOTYPE 1 FUNCTIONS ///////////////////////

int speedX = 0, speedY = 0, rSpeedZ = 0; // m/s and rad/s
const float L1 = 1, L2 = 1; //dimensions in m
const float SpeedtoRad = 1.47; 

unsigned int msCount2 = 0; //millisecond count on timer 2

/**
 * updates the x and y speeds of the robot according to the global speed variables
 */
void DriveXYZ() {
  left_front_motor.writeMicroseconds(1500 + speedX + speedY + (L1+L2)*rSpeedZ);
  left_rear_motor.writeMicroseconds(1500 + speedX - speedY + (L1+L2)*rSpeedZ);
  right_rear_motor.writeMicroseconds(1500 - speedX - speedY  + (L1+L2)*rSpeedZ);
  right_front_motor.writeMicroseconds(1500 - speedX + speedY + (L1+L2)*rSpeedZ);
}

void RotateDeg(float deg = 0.0){
  float error = 0.0;
  rSpeedZ = 100;
 
  while(error < deg){
    DriveXYZ();
    error = rSpeedZ * (180.0/PI) * (SpeedtoRad/100.0) * (msCount2/1000.0); 
    SerialCom->println(deg);
  }

  rSpeedZ = 0;
  DriveXYZ();
  SerialCom->println("stop");
}

void UpdateUltra() {
 // Find current sensor reading
  CurrentSensor[4] = HC_SR04_range();
  
  double total;
  // Find average reading from past 10 readings
  total = 0;
  for (int m = 0; m < 10; m++) {
    total = PrevSensor[4][m] + total;
  }
  
  Average[4] = total/10;

  // Add current reading to old readings
  PrevSensor[4][timer2i] = CurrentSensor[4]; 
  timer2i = (timer2i+1)%10;
}

void RotateToMinDist(){
  CLRotateDeg(-30);
  float degDesired = 60.0f;

  float distances[200];
  float angles[200] = {0};

  float degDriven = 0; 

  const int Kp = 5, Ki = 0, Kd = 0;

  const float errorTolerance = 0; 
  float error = degDesired - degDriven;

  int msCount = 0;
  int i;

  float prevAngle = 0; 

  for(i = 0; i < 20; i++) distances[i] = HC_SR04_range();
  
  int deltaT = millis();
  while(error > errorTolerance){
    float v = GYRO_reading();
    deltaT = (millis() - deltaT);
    degDriven += v*(deltaT-0.6)/1000.0;

    deltaT = millis();
    error = degDesired - degDriven;

    float effort = abs(Kp*error);
  
    
    if(effort < 80) effort = 80; 
    if(effort > 200) effort = 200; 

    speed_val = abs(effort); 

    if(error > 0) cw();
    else ccw();

    msCount+= 10;
    if(msCount == 10){
      msCount = 0;
      distances[i] = HC_SR04_range();
      angles[i] = KalmanFilter(degDriven, prevAngle);
      prevAngle = degDriven;
      i++;
    }

    delay(10);
  }
  stop();

  for(int j = 2; j < i-2; j++){
    if(distances[j] == 0){
      int sum = distances[j-1] + distances[j-2] + distances[j+1] + distances[j+2];
      distances[j] = sum/4;
    }
  }

  for(int j = 5; j < i-5; j++){
    float sum = 0;
    for(int k = -5; k < 5; k++){
      sum += distances[j+k];
    }
    distances[j] = sum/10;
  }

  for(int j = 1; j < i; j++){
    //angles[j] = kalmanfilter
  }

  BluetoothSerial.println(i);

  int minIndex = 10;
  for(int j = 10; j < i; j++){
    BluetoothSerial.print(distances[j]);
    BluetoothSerial.print(", ");
    BluetoothSerial.println(angles[j]);
    minIndex = (distances[j] < distances[minIndex] && distances[j] != 0) || distances[minIndex] == 0 ? j : minIndex;
  }

  float newAngle = angles[minIndex] - 60;
  
  //if(newAngle > 180) newAngle -= 360;
  newAngle = newAngle - 3; //abs(newAngle)/newAngle;

  BluetoothSerial.print("Min Angle: ");
  BluetoothSerial.print(newAngle);
  BluetoothSerial.print(" at ");
  BluetoothSerial.print(distances[minIndex]);

  delay(500);

  CLRotateDeg(newAngle);
}

void LocateCorner2(){
  float degDesired = 360.0f;

  float distances[200];
  float angles[200] = {0};

  float degDriven = 0; 

  const int Kp = 4, Ki = 0, Kd = 0;

  const float errorTolerance = 0; 
  float error = degDesired - degDriven;

  int msCount = 0;
  int i;

  float prevAngle = 0; 

  for(i = 0; i < 10; i++) distances[i] = HC_SR04_range();
  
  int deltaT = millis();
  while(error > errorTolerance){
    float v = GYRO_reading();
    deltaT = (millis() - deltaT);
    degDriven += v*(deltaT-0.6)/1000.0;

    deltaT = millis();
    error = degDesired - degDriven;

    float effort = abs(Kp*error);
  
    
    if(effort < 80) effort = 80; 
    if(effort > 200) effort = 200; 

    speed_val = abs(effort); 

    if(error > 0) cw();
    else ccw();

    msCount+= 10;
    if(msCount == 50){
      msCount = 0;
      distances[i] = HC_SR04_range();
      angles[i] = KalmanFilter(degDriven, prevAngle);
      prevAngle = degDriven;
      i++;
    }

    delay(10);
  }
  stop();

  for(int j = 2; j < i-2; j++){
    if(distances[j] == 0){
      int sum = distances[j-1] + distances[j-2] + distances[j+1] + distances[j+2];
      distances[j] = sum/4;
    }
  }

  for(int j = 5; j < i-5; j++){
    float sum = 0;
    for(int k = -5; k < 5; k++){
      sum += distances[j+k];
    }
    distances[j] = sum/10;
  }

  for(int j = 1; j < i; j++){
    //angles[j] = kalmanfilter
  }

  BluetoothSerial.println(i);

  int minIndex = 10;
  for(int j = 10; j < i; j++){
    BluetoothSerial.print(distances[j]);
    BluetoothSerial.print(", ");
    BluetoothSerial.println(angles[j]);
    minIndex = (distances[j] < distances[minIndex] && distances[j] != 0) || distances[minIndex] == 0 ? j : minIndex;
  }

  float newAngle = angles[minIndex];
  
  if(newAngle > 180) newAngle -= 360;

  BluetoothSerial.print("Min Angle: ");
  BluetoothSerial.print(newAngle);
  BluetoothSerial.print(" at ");
  BluetoothSerial.print(distances[minIndex]);

  delay(500);

  CLRotateDeg(newAngle - 10);

  delay(500);

  float oppositeAngle = (newAngle + 180.0);

  BluetoothSerial.print("| Op Real Angle: ");
  int j;
  for(j = 10; j < i; j++){
    if(angles[j] > oppositeAngle) break;
  }
  //BluetoothSerial.print(angles[j]);

  float crossdist = distances[j] + distances[minIndex];

  //BluetoothSerial.print(" | Cross Dist: ");
  //BluetoothSerial.print(crossdist);

  delay(500); 
  
  if(crossdist < 140){
    CLRotateDeg(90);
  }
  
  //RotateToMinDist();

  MoveToCorner();
  MoveToCorner();


}

void CLRotateDeg(float degDesired){
  double degDriven = 0; 

  const int Kp = 4, Ki = 0, Kd = 0;

  const float errorTolerance = 0; 
  float prevError = 0, error = degDesired - degDriven;

  int deltaT = millis(); //ms
  const int direct = degDesired/abs(degDesired);

  while(error*direct  > errorTolerance){
    float v = GYRO_reading();

    deltaT = millis() - deltaT;
    degDriven += v*(deltaT-0.6)/1000.0;
    deltaT = millis();

    error = degDesired - degDriven;

    float effort = abs(Kp*error);

    if(effort < 80) effort = 80; 
    if(effort > 300) effort = 300; 

    speed_val = effort; 

    if(direct > 0) cw();
    else ccw();

    prevError = error;

    delay(10);
  }

  stop();
}


void DriveToDist(float distance){
  forward();
  speed_val = 400;
  int count  = 0;
  float ultrasonic;
  // Stop driving when closer than distance cm
  while (count <= 20) {
    ultrasonic = HC_SR04_range();
    if (ultrasonic <= distance) {
      count = count + 1;
    }
  }
  stop();
}

void MoveToCorner() {
  //give ultrasonic average time to settle
  for(int i = 0; i<50; i++) UpdateSensors();
  DriveStraight(10, true);
    CLRotateDeg(90);
}

void DriveStraight(float ForwardDistance, bool direct) {
  // ForwardDistance is distance from wall to drive to
  // direct is true if forwards, false if backwards
  float Kz = 2;
  float Fz;
  float Fx = 7;
  float Fy = 0;
  float rotation;
  float ThetaOne, ThetaTwo, ThetaThree, ThetaFour;
  float timee = 0;
  float timeEffect = 0;
  
  //Robot Dimensions, specific measurements are shown in our notes
  float Rw = 0.022; //Unit is metres
  float L = 0.09; //Unit is metres
  float t = 0.09; //Unit is metres
  float Constant = 1 / Rw;
  
  for (int i = 0; i < 20; i++) {
    UpdateSensors(); 
  } // Update the sensor readings
  // Average[0] is Left Mid
  // Avergae[1] is Right Mid
  // Average[2] is Left Long
  // Average[3] is Right Long
  // Average[4] is ultrasonic;
  
  if (direct == false) {
    Fx = -Fx;
    while(ForwardDistance >= Average[4]) { //Drive Backwards
      
      if (timee <= 250) { // Ramp up the power from 0 to 100
      timee = timee + 1;
      timeEffect = timee/500;
      }
      UpdateSensors();
      SLAM(SLAMdirect);
      rotation = GYRO_reading();
      // Positive is clockwise
      Fz = Kz * rotation - 2.02; // Turning force

      //Calculate Motor Speed
      ThetaOne = timeEffect * Constant * (Fx + Fy - (L + t) * Fz);
      ThetaTwo = timeEffect * Constant * (Fx - Fy + (L + t) * Fz);
      ThetaThree = timeEffect * Constant * (Fx - Fy - (L + t) * Fz);
      ThetaFour = timeEffect * Constant * (Fx + Fy + (L + t) * Fz);
      
      // Calculate Motor Power
      left_front_motor.writeMicroseconds(1500 + ThetaOne);
      right_front_motor.writeMicroseconds(1500 - ThetaTwo);
      left_rear_motor.writeMicroseconds(1500 + ThetaThree);
      right_rear_motor.writeMicroseconds(1500 - ThetaFour); 
    }  
  }

  else {
    while(ForwardDistance <= Average[4]) { //Drive Forwards
      if (timee <= 250) { // Ramp up the power from 0 to 100
      timee = timee + 1;
      timeEffect = timee/500;
      }
      UpdateSensors();
      SLAM(SLAMdirect);
      rotation = GYRO_reading();
      // Positive is clockwise
      Fz = Kz * rotation + 0.02; // Turning force

      //Calculate Motor Speed
      ThetaOne = timeEffect * Constant * (Fx + Fy - (L + t) * Fz);
      ThetaTwo = timeEffect * Constant * (Fx - Fy + (L + t) * Fz);
      ThetaThree = timeEffect * Constant * (Fx - Fy - (L + t) * Fz);
      ThetaFour = timeEffect * Constant * (Fx + Fy + (L + t) * Fz);
      
      // Calculate Motor Power
      left_front_motor.writeMicroseconds(1500 + ThetaOne);
      right_front_motor.writeMicroseconds(1500 - ThetaTwo);
      left_rear_motor.writeMicroseconds(1500 + ThetaThree);
      right_rear_motor.writeMicroseconds(1500 - ThetaFour); 
    }

  }
    while(timee >= 0) {
    ThetaOne = ThetaOne * timee / 500;
    ThetaTwo = ThetaTwo * timee / 500;
    ThetaThree = ThetaThree * timee / 500;
    ThetaFour = ThetaFour * timee / 500;

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);
    timee = timee - 1;
  }
  stop();
}

void DriveSide(DIRECTION direct, int time) {
  float Kz = 1;
  float Fz;
  float Fx = 0;
  float Fy = 5;
  float rotation;
  float ThetaOne, ThetaTwo, ThetaThree, ThetaFour;
  int count = 0;
  int timee = 1000;
  
  //Robot Dimensions, specific measurements are shown in our notes
  float Rw = 0.022; //Unit is metres
  float L = 0.09; //Unit is metres
  float t = 0.09; //Unit is metres
  float Constant = 1 / Rw;
  
  for (int i = 0; i < 20; i++) {
    UpdateSensors(); 
  } // Update the sensor readings
  // Average[0] is Left Mid
  // Avergae[1] is Right Mid
  // Average[2] is Left Long
  // Average[3] is Right Long
  // Average[4] is ultrasonic

    if(direct == LEFT) {
  Fy = -Fy;
    }

    while(count < time) { 
      UpdateSensors();
      SLAM(SLAMdirect);
      
      rotation = GYRO_reading();
      // Positive is clockwise
      Fz = Kz * rotation; // Turning force
      
      //Calculate Motor Speed
      ThetaOne = Constant * (Fx + Fy - (L + t) * Fz);
      ThetaTwo = Constant * (Fx - Fy + (L + t) * Fz);
      ThetaThree = Constant * (Fx - Fy - (L + t) * Fz);
      ThetaFour = Constant * (Fx + Fy + (L + t) * Fz);
      
      // Calculate Motor Power
      left_front_motor.writeMicroseconds(1500 + ThetaOne);
      right_front_motor.writeMicroseconds(1500 - ThetaTwo);
      left_rear_motor.writeMicroseconds(1500 + ThetaThree);
      right_rear_motor.writeMicroseconds(1500 - ThetaFour);
      
      delay(10);
      count = count + 1;
    }
    
    while(timee >= 0) {
    ThetaOne = ThetaOne * timee / 1000;
    ThetaTwo = ThetaTwo * timee / 1000;
    ThetaThree = ThetaThree * timee / 1000;
    ThetaFour = ThetaFour * timee / 1000;

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);
    timee = timee - 1;
  }
  stop();
}

void GoEdge(float SideDistance, DIRECTION direct, int LongOrMid) {
  //LongOrMid
  //2 = Long range sensor
  //0 = Mid range sensor
  float Kx = 0;
  float Ky = 0.75;
  float Kz = 0;
  float Fx, Fy, Fz;
  float Confidence;
  int count = 0;

  float Kyi = 0.01;
  float integralError = 0;

  for (int i = 0; i < 20; i++) {
    UpdateSensors(); 
  } // Update the sensor readings
  // Average[0] is Left Mid
  // Avergae[1] is Right Mid
  // Average[2] is Left Long
  // Average[3] is Right Long
  // Average[4] is ultrasonic
  int Left = 0 + LongOrMid;
  int Right = 1 + LongOrMid;

  //Robot Dimensions, specific measurements are shown in our notes
  float Rw = 0.022; //Unit is metres
  float L = 0.09; //Unit is metres
  float t = 0.09; //Unit is metres
  float Constant = 1 / Rw;

  float ThetaOne, ThetaTwo, ThetaThree, ThetaFour;

  float CurrentIRReading, PreviousIRReading;
  if (direct == LEFT) {
    CurrentIRReading = Average[Left];
  } else {
    CurrentIRReading = Average[Right];
  }

  // LEFT SIDE
  while (count < 20 && direct == LEFT) {
    UpdateSensors();
    SLAM(SLAMdirect);
    //BluetoothSerial.println(CurrentIRReading);

    PreviousIRReading = CurrentIRReading;
    CurrentIRReading = Average[Left];

    if ((SideDistance - CurrentIRReading) > 2) {
      integralError = 0;
    }
    else {
      integralError = integralError + (SideDistance - CurrentIRReading);
    }   
    
    Fy = Ky * (SideDistance - CurrentIRReading) + Kyi * integralError;
    Fz = 0;
    Fx = 0;
    
    //Calculate Motor Speed
    ThetaOne = Constant * (Fx + Fy - (L + t) * Fz);
    ThetaTwo = Constant * (Fx - Fy + (L + t) * Fz);
    ThetaThree = Constant * (Fx - Fy - (L + t) * Fz);
    ThetaFour = Constant * (Fx + Fy + (L + t) * Fz);

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);


    if ((CurrentIRReading < SideDistance + 1) && (CurrentIRReading > SideDistance - 1)) {
      count = count + 1;
    } else {count = 0;}
  }

    // RIGHT SIDE
  while (count < 20 && direct == RIGHT) {
    UpdateSensors();
    SLAM(SLAMdirect);

    PreviousIRReading = CurrentIRReading;
    CurrentIRReading = Average[Right];

    if ((SideDistance - CurrentIRReading) > 2) {
      integralError = 0;
    }
    else {
      integralError = integralError + (SideDistance - CurrentIRReading);
    }   
    
    Fy = -(Ky * (SideDistance - CurrentIRReading) + Kyi * integralError);
    Fz = 0;
    Fx = 0;
    
    //Calculate Motor Speed
    ThetaOne = Constant * (Fx + Fy - (L + t) * Fz);
    ThetaTwo = Constant * (Fx - Fy + (L + t) * Fz);
    ThetaThree = Constant * (Fx - Fy - (L + t) * Fz);
    ThetaFour = Constant * (Fx + Fy + (L + t) * Fz);

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);

    if (CurrentIRReading < SideDistance + 1 && CurrentIRReading > SideDistance - 1) {
      count = count + 1;
    } else {count = 0;}
  }
  
}

void FollowEdge(float ForwardDistance, float SideDistance, DIRECTION direct, int LongOrMid) {
  //LongOrMid
  //2 = Long range sensor
  //0 = Mid range sensor
  float Kx = 0.5;
  float Ky = 0.75;
  float Kz = 120;
  float Fx, Fy, Fz;
  float Confidence;

  float timee = 0;
  float timeEffect = 0;
  
  float Kyi = 0;
  float integralError = 0;

  GoEdge(SideDistance, direct, LongOrMid);

  for (int i = 0; i < 20; i++) {
    UpdateSensors(); 
  } // Update the sensor readings
  // Average[0] is Left Mid
  // Avergae[1] is Right Mid
  // Average[2] is Left Long
  // Average[3] is Right Long
  // Average[4] is ultrasonic
  int Left = 0 + LongOrMid;
  int Right = 1 + LongOrMid;

  //Robot Dimensions, specific measurements are shown in our notes
  float Rw = 0.022; //Unit is metres
  float L = 0.09; //Unit is metres
  float t = 0.09; //Unit is metres
  float Constant = 1 / Rw;

  float ThetaOne, ThetaTwo, ThetaThree, ThetaFour;

  float CurrentIRReading, PreviousIRReading;
  if (direct == LEFT) {
    CurrentIRReading = Average[Left];
  } else {
    CurrentIRReading = Average[Right];
  }
  
  while (Average[4] >= ForwardDistance && direct == LEFT) {
    if (timee < 1000) { // Ramp up the power from 0 to 100
      timee = timee + 1;
      timeEffect = timee/1000;
    }
    
    UpdateSensors();
    SLAM(SLAMdirect);

    PreviousIRReading = CurrentIRReading;
    CurrentIRReading = Average[Left];

    if ((SideDistance - CurrentIRReading) > 1.5) {
      integralError = 0;
    }
    else {
      integralError = integralError + (SideDistance - CurrentIRReading);
    }   
    
    Fy = Ky * (SideDistance - CurrentIRReading) + Kyi * integralError;

    /*
    if ((CurrentIRReading - PreviousIRReading > 0) && (SideDistance - CurrentIRReading > 0)) { // Gap is growing and above goal
      Fz = 0;
    }
    if ((CurrentIRReading - PreviousIRReading) > 0 && (SideDistance - CurrentIRReading < 0)) { // Gap is growing and less than goal
      Fz = Kz * (CurrentIRReading - PreviousIRReading);
    }
    if ((CurrentIRReading - PreviousIRReading < 0) && (SideDistance - CurrentIRReading > 0)) { // Gap is shrinking and above goal
      Fz = Kz * (CurrentIRReading - PreviousIRReading);

    }
    if ((CurrentIRReading - PreviousIRReading < 0) && (SideDistance - CurrentIRReading < 0)) { // Gap is shrinking and less than goal
      Fz = 0;
    }*/
    Fz = Kz * (CurrentIRReading - PreviousIRReading) + 2;

    Fx = Kx / (abs((CurrentIRReading - PreviousIRReading) * (SideDistance - CurrentIRReading)) + 0.01);
    
    //Calculate Motor Speed
    ThetaOne = timeEffect * Constant * (Fx + Fy - (L + t) * Fz);
    ThetaTwo = timeEffect * Constant * (Fx - Fy + (L + t) * Fz);
    ThetaThree = timeEffect * Constant * (Fx - Fy - (L + t) * Fz);
    ThetaFour = timeEffect * Constant * (Fx + Fy + (L + t) * Fz);

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);

  }

  while (Average[4] >= ForwardDistance && direct == RIGHT) {
    if (timee < 1000) { // Ramp up the power from 0 to 100
      timee = timee + 1;
      timeEffect = timee/1000;
    }
    UpdateSensors();
    SLAM(SLAMdirect);
    
    // Calculate Fz
    PreviousIRReading = CurrentIRReading;
    CurrentIRReading = Average[Right];

    if ((SideDistance - CurrentIRReading) > 1.5) {
      integralError = 0;
    } else {
      integralError = integralError + (SideDistance - CurrentIRReading);
    }   
    /*
    if (CurrentIRReading - PreviousIRReading > 0 && SideDistance - CurrentIRReading > 0) { // Gap is growing and above goal
      Fz = 0;
    }
    if (CurrentIRReading - PreviousIRReading > 0 && SideDistance - CurrentIRReading < 0) { // Gap is growing and less than goal
      Fz = -Kz * (CurrentIRReading - PreviousIRReading);
    }
    if (CurrentIRReading - PreviousIRReading < 0 && SideDistance - CurrentIRReading > 0) { // Gap is shrinking and above goal
      Fz = -Kz * (CurrentIRReading - PreviousIRReading);
    }
    if (CurrentIRReading - PreviousIRReading < 0 && SideDistance - CurrentIRReading < 0) { // Gap is shrinking and less than goal
      Fz = 0;
    }*/
    Fz = Kz * (CurrentIRReading - PreviousIRReading) + 1.5;
    
    Fy = -(Ky * (SideDistance - CurrentIRReading) + Kyi * integralError);

    Fx = (Kx / (abs((CurrentIRReading - PreviousIRReading) * (SideDistance - CurrentIRReading)) + 0.01));
    
    //Calculate Motor Speed
    ThetaOne = timeEffect * Constant * (Fx + Fy - (L + t) * Fz);
    ThetaTwo = timeEffect * Constant * (Fx - Fy + (L + t) * Fz);
    ThetaThree = timeEffect * Constant * (Fx - Fy - (L + t) * Fz);
    ThetaFour = timeEffect * Constant * (Fx + Fy + (L + t) * Fz);

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);
    
  }
  while(timee >= 0) {
    ThetaOne = ThetaOne * timee / 1000;
    ThetaTwo = ThetaTwo * timee / 1000;
    ThetaThree = ThetaThree * timee / 1000;
    ThetaFour = ThetaFour * timee / 1000;

    // Calculate Motor Power
    left_front_motor.writeMicroseconds(1500 + ThetaOne);
    right_front_motor.writeMicroseconds(1500 - ThetaTwo);
    left_rear_motor.writeMicroseconds(1500 + ThetaThree);
    right_rear_motor.writeMicroseconds(1500 - ThetaFour);
    timee = timee - 1;
  }
  GoEdge(SideDistance, direct, LongOrMid);
  stop();
 }

// Rotates the robot by rougly 180 degrees
void Rotate180(void) {
  cw(); // Send motors clockwise
  delay(5000);
  stop();
}

double FindCloseEdge(void) {
  
}

#ifndef NO_READ_GYRO
float gyroSupplyVoltage = 5;  
float gyroZeroVoltage = 0;   

float gyroSensitivity = 0.0065;// 0.007 + 0.007*0.04;       
float rotationThreshold = 1.5;  

float currentAngle = 0;

bool GYRO_calibrate(){
  float sum = 0;
  int n = 100; //number of measurements 

  for(int i = 0; i < n; i++){
    sum += gyroSupplyVoltage*(analogRead(GYRO_PIN)/1023.0);
  }

  gyroZeroVoltage = sum/n; 

  return true;
}


float prevAngularVelocity = 0;

float GYRO_reading()
{
  
  float gyroRate = (analogRead(GYRO_PIN)/1023.0)*gyroSupplyVoltage - gyroZeroVoltage;
  float angularVelocity = gyroRate/gyroSensitivity;

  angularVelocity = abs(angularVelocity)>rotationThreshold ? angularVelocity : 0;
  //angularVelocity = KalmanFilter(angularVelocity, prevAngularVelocity);

  prevAngularVelocity = angularVelocity; 
  return angularVelocity;
  
}
#endif

////////////////// SENSOR FUNCTIONS /////////////////////////////////
double KalmanFilter(double rawdata, double prev_est){
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}

double IRSensorReading(IR_SENSOR sensor){
  int val;
  double temp, est, var;
  
  switch(sensor){
  case LEFT_MID: 
      val = analogRead(MID_RANGE_LEFT_PIN); //Reading raw value from analog port
      temp = Mid_Left_Value * pow(val,Mid_Left_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      return est;
  case LEFT_LONG:
      val = analogRead(LONG_RANGE_LEFT_PIN); //Reading raw value from analog port
      temp = Long_Left_Value * pow(val,Long_Left_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est); 
      return est;
  case RIGHT_MID:
      val = analogRead(MID_RANGE_RIGHT_PIN); //Reading raw value from analog port
      temp = Mid_Right_Value * pow(val,Mid_Right_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      return est;
  case RIGHT_LONG:
      val = analogRead(LONG_RANGE_RIGHT_PIN); //Reading raw value from analog port
      temp = Long_Right_Value * pow(val,Long_Right_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      return est;
  }
}

void UpdateSensors() {
  // Take and compare new IR sensor readings with previous sensor readings
  
  // Find current sensor readings
  CurrentSensor[0] = IRSensorReading(LEFT_MID);
  CurrentSensor[1] = IRSensorReading(RIGHT_MID);
  CurrentSensor[2] = IRSensorReading(LEFT_LONG);
  CurrentSensor[3] = IRSensorReading(RIGHT_LONG);
  CurrentSensor[4] = HC_SR04_range();
  //CurrentSensor[5] = GYRO_reading();
  //CurrentSensor

  double total;
  // Find average reading from past 10 readings
  for (int n = 0; n <= 4; n++) {
    total = 0;
    for (int m = 0; m < 10; m++) {
      total = PrevSensor[n][m] + total;
    }
    Average[n] = total/10;
  }
  
  // Add current reading to old readings
  PrevSensor[0][timer2i] = CurrentSensor[0];
  PrevSensor[1][timer2i] = CurrentSensor[1];
  PrevSensor[2][timer2i] = CurrentSensor[2];
  PrevSensor[3][timer2i] = CurrentSensor[3];
  PrevSensor[4][timer2i] = CurrentSensor[4];
  //PrevSensor[5][timer2i] = CurrentSensor[5];
  
  timer2i = (timer2i+1)%10;
}

void SLAM(DIRECTION direct){
  
  float IRDifference, UltrasonicDifference;
  //Update Sensor readings
  int SideChange;
    
  //This function will be used to create a map and port it externally
  if (MapRowCounter == 0){//The SLAM function should only be run once the robot is in the corner, so it can be initialised as the 0,0 position
    Map[0][0] = 0;
    Map[0][1] = 0;

    PastDirect = direct;
    if (direct == LEFT) {
      PastIRReadingSLAM = Average[2];
    }
    else if (direct == RIGHT) {
      PastIRReadingSLAM = Average[3];
    }
    PastUltrasonicSLAM = Average[4];
    
    BluetoothSerial.print(Map[0][0]);
    BluetoothSerial.print(", ");
    BluetoothSerial.println(Map[0][1]);
    MapRowCounter++;
    PastMapRowCounter = MapRowCounter - 1;  
    return;
  }

  //Use the ultrasound sensor and the IR sensor (direction given using the direct input) to gain information about the environment and map it to the 2D array
  //The difference between the past and current measurements will dictace how far the robot has moved
  if (direct == LEFT) {
    IRReadingSLAM = Average[2];
  }

  else if (direct == RIGHT) {
    IRReadingSLAM = Average[3];
  }
  UltrasonicSLAM = Average[4];

  //The mapping method for the y direction is different depending on which half of the table the robot is on, the x co-ordinate method is unaffected
  UltrasonicDifference = PastUltrasonicSLAM - UltrasonicSLAM;
  //Ultrasonic (X Direction)
  Map[1][0] = Map[0][0] + UltrasonicDifference;  
  
  if ((direct == PastDirect) && (SideChange == 0)){
    //Robot is still in the first half of the mapping phase, calculations are done in the following code
    //Use the current readings and the past readings to calculate how far the robot has moved
    IRDifference = PastIRReadingSLAM - IRReadingSLAM;
  
    //take the previous co-ordinates and add the differences for the new map measurements. Note that the difference will already be positive or negative to account for direction travelled.
    //IR Sensor (Y Direction)
    Map[1][1] = Map[0][1] + IRDifference;
    PastDirect = direct;
  }

  else if ((direct != PastDirect) || (SideChange == 1)){
    //IR Difference is calculated in a different way, do not update the PastDirect value in this section so that this conditional is fufilled for the rest of the function
    IRDifference = 40 - IRReadingSLAM; //120cm wide table, at the second half IR sensor is reading how far away it is from the 120cm mark instead of the origin.
    Map[1][1] = Map[0][1] + IRDifference; 
    SideChange = 1;
  }
  
 
  
  BluetoothSerial.print(Map[1][0]);
  BluetoothSerial.print(", ");
  BluetoothSerial.println(Map[1][1]);
  
  /*
  SerialCom->print(Map[MapRowCounter][0]);
  SerialCom->print(", ");
  SerialCom->println(Map[MapRowCounter][1]);
  SerialCom->println("Test");
  */
  
  //Adjust Past Values for next function call
  MapRowCounter++;
  Map[0][0]= Map[1][0];
  Map[0][1] = Map[1][1];
  PastMapRowCounter = MapRowCounter - 1;
  PastIRReadingSLAM = IRReadingSLAM;
  PastUltrasonicSLAM = UltrasonicSLAM;
  return;
  
}
///////////////////////////////////////////////////////////////////

STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  
  static unsigned long previous_millis;

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();

  #ifndef NO_READ_GYRO
      //GYRO_reading();
  #endif
  
  #ifndef NO_READ_IR
      IR_reading(LEFT_MID);
      IR_reading(LEFT_LONG);
      IR_reading(RIGHT_MID);
      IR_reading(RIGHT_LONG);
  #endif
  
  #ifndef NO_HCSR04
      HC_SR04_range();
  #endif
  
  #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
  #endif
  }

  // PROTOTYPE 1 //////////////////////
 // Average[2] is Left Long
 // Average[3] is Right Long
 GYRO_calibrate();

 //CLRotateDeg(90);
  //RotateToMinDist();
  //delay(2000);
 //BluetoothSerial.println("GO ROBOT GO");
 /////////////////////////////////////////////FIND CORNER/////////
  LocateCorner2();
  delay(250);

  //////////////////////////


 int SLAMCounter = 0;
 FollowEdge(15, 6, LEFT, 0); //Second input is side distance
 DriveSide(RIGHT, 30); //Change second input to change how long it shifts for 
 RotateToMinDist();
 DriveStraight(165, false); //False means drive backwards
 DriveSide(RIGHT, 30); //Change second input to change how long it shifts for 

 for (int i = 0; i < 10; i++) {
    UpdateSensors(); 
  }

 while(j < 3){
  
    if (SLAMCounter >= 2) { //Change depending on how many loops are needed to get to the middle
      SLAMdirect == RIGHT;
    }
    DriveStraight(20, true); //False means drive backwards
    DriveSide(RIGHT, 30); //Change second input to change how long it shifts for 
    DriveStraight(165, false); //False means drive backwards
    DriveSide(RIGHT, 30); //Change second input to change how long it shifts for
    SLAMCounter++; 
    j = j + 1;
    for (int i = 0; i < 10; i++) {
    UpdateSensors(); 
  }      
 }

   GoEdge(15, RIGHT, 2);
   FollowEdge(20, 6
   , RIGHT, 0); //Second input is side distance
   disable_motors();
   delay(50000);

   
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
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
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HCSR04
float HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      //SerialCom->println("HC-SR04: NOT found");
      return float(Average[4]);
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      //SerialCom->println("HC-SR04: Out of range");
      return float(Average[4]);
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    //SerialCom->println("HC-SR04: Out of range");
  } else {
    //SerialCom->print("HC-SR04:");
    //SerialCom->print(cm);
    //SerialCom->println("cm");
  }

  return cm;
}
#endif

#ifndef NO_READ_IR
void IR_reading(IR_SENSOR sensor)
{
  int val;
  double temp, est, var;
  //SerialCom->print("IR Sensor:");
  switch (sensor)
  {
    case LEFT_MID:  //MID_RANGE_LEFT_PIN
      //SerialCom->println("Mid Left IR Sensor: ");
      val = analogRead(MID_RANGE_LEFT_PIN); //Reading raw value from analog port
      temp = Mid_Left_Value * pow(val,Mid_Left_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      //SerialCom->print("Unfiltered Value: ");
      //SerialCom->print(temp); 
      //SerialCom->println(" Cm");
      //SerialCom->print("Filtered Value: ");
      //SerialCom->print(est); 
      //SerialCom->println(" Cm");
      break;
    case LEFT_LONG: //LONG_RANGE_LEFT_PIN
      //SerialCom->println("Long Left IR Sensor: ");
      val = analogRead(LONG_RANGE_LEFT_PIN); //Reading raw value from analog port
      temp = Long_Left_Value * pow(val,Long_Left_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      //SerialCom->print("Unfiltered Value: ");
      //SerialCom->print(temp); 
      //SerialCom->println(" Cm");
      //SerialCom->print("Filtered Value: ");
      //SerialCom->print(est); 
      //SerialCom->println(" Cm");
      break;
    case RIGHT_LONG: //LONG_RANGE_RIGHT_PIN
      //SerialCom->print("Long Right IR Sensor: ");
      val = analogRead(LONG_RANGE_RIGHT_PIN); //Reading raw value from analog port
      //SerialCom->println(val);
      temp = Long_Right_Value * pow(val,Long_Right_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      //SerialCom->print("Unfiltered Value: ");
      //SerialCom->print(temp); 
      //SerialCom->println(" Cm");
      //SerialCom->print("Filtered Value: ");
      //SerialCom->print(est); 
      //SerialCom->println(" Cm");
      break;
     case RIGHT_MID: //MID_RANGE_RIGHT_PIN
      //SerialCom->println("Mid Right IR Sensor: ");
      val = analogRead(MID_RANGE_RIGHT_PIN); //Reading raw value from analog port
      temp = Mid_Right_Value * pow(val,Mid_Right_Exponent); //Convert to mm distance based on sensor calibration
      est = KalmanFilter(temp, last_est);
      //SerialCom->print("Unfiltered Value: ");
      //SerialCom->print(temp); 
      //SerialCom->println(" Cm");
      //SerialCom->print("Filtered Value: ");
      //SerialCom->print(est); 
      //SerialCom->println(" Cm");
      break;
  }
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_front_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}
