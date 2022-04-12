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
#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

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
double CurrentSensor[4 + 1]; //IR 1-4 //Ultrasonic //Gyro
double Average[4 + 1];
double PrevSensor[4 + 1][10];
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

//Long Right Sensor uses a logarithmic equation instead of a power equation, so the constants are defined differently, they need to be calculated every loop, as the changing reading is right in the middle of the equation

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
  msCount2 = 0;
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

void CLRotateDeg(float desiredPos){
  float pos = 0, rate, acc; //integral, val, derivative 
  const int Kp = 0, Ki = 3, Kd = 0;

  short timeStopped = 0;
  const short deltaT = 10; //ms
  const short toleranceSettleTime = 1000;

  const float errorTolerance = 0; 
  float prevRate = 0;
  float error = desiredPos - pos;

  auto saturateEffort =  [&] (float *effort){
      *effort = (*effort < 75) ? 75 : *effort;
      *effort = (*effort < 500) ? 500 : *effort;
  };

  while(abs(error) > errorTolerance && timeStopped > toleranceSettleTime){
    rate =  GYRO_reading(); // rad/s
    acc = (rate - prevRate)*1000.0/deltaT; // rad/s^2
    pos += rate*deltaT/1000.0; // rad

    error = desiredPos - pos;
    
    float effort = Kp*rate + Ki*pos + Kd*acc;
    effort = abs(effort);
    saturateEffort(&effort);

    if(error > 0) cw();
    else ccw();

    prevRate = rate;

    if(abs(error) < errorTolerance) timeStopped += deltaT;
    else timeStopped = 0;

    delay(deltaT);
  }
  stop();
}

void LocateCorner(void) {

  const short n = 72; //division of measurement circle (n measurements for 360deg). most accurate as large multiple of 4
  float distance[n];
  int minIndex = 0; 

  //get distance from array with circular indices (if i > n, return value val from next rotation of array)
  auto getDist = [&] (int i){
    i = i - (i%n)*n;
    return distance[i];
  }; 

  //give ultrasonic average time to settle
  for(int i = 0; i<100; i++) UpdateSensors(); 

  //rotate car, populate measurement array and find min distance to wall
  for(int i = 0; i < n; i++){
    CLRotateDeg((360/(n)));
    distance[i] = Average[4]; 
    BluetoothSerial.print('%f ,', Average[4]); 
    minIndex = distance[i] < distance[minIndex] || distance[minIndex] == 0 ? i : minIndex;
  }

  BluetoothSerial.print('\n min dist at %f \176 \n\n', minIndex*(360.0/n));

  delay(2000);

  //rotate to minimum dist to wall
  CLRotateDeg(minIndex*(360.0/n)); 

  //correct if not pointing at the width (short) wall
  if((getDist(minIndex) + getDist(minIndex + (int)(n/2))) < 
    (getDist(minIndex + (int)(n/4)) + getDist(minIndex + (int)(3*n/4))))  
      RotateDeg(90);
}

void DriveToDist(float desiredPos){
  float integral = 0, pos, rate; //integral, val, derivative 
  const int Kp = 0, Ki = 3, Kd = 0;

  short timeStopped = 0;
  const short deltaT = 10; //ms
  const short toleranceSettleTime = 1000;

  const float errorTolerance = 0; 
  float prevPos = 0;
  float error = desiredPos - pos;

  auto saturateEffort =  [&] (float *effort){
      *effort = (*effort < 75) ? 75 : *effort;
      *effort = (*effort < 500) ? 500 : *effort;
  };

  while(abs(error) > errorTolerance && timeStopped > toleranceSettleTime){
    pos = Average[4];
    rate = (pos - prevPos)*1000.0/deltaT; // cm/s
    integral += pos*deltaT/1000.0; // cm

    error = desiredPos - pos;
    
    float effort = Kp*pos + Ki*integral + Kd*rate;
    effort = abs(effort);
    saturateEffort(&effort);

    if(error > 0) forward;
    else reverse;

    prevPos = pos;

    if(abs(error) <= errorTolerance) timeStopped += deltaT;
    else timeStopped = 0;

    delay(deltaT);
  }
  stop();
}

void MoveToCorner(float distance) {
  DriveToDist(10);
  CLRotateDeg(90);
  DriveToDist(10);
  CLRotateDeg(90);
}

void AlignEdge(void) {
  
}

void FollowEdge(int ForwardDistance, int SideDistance, DIRECTION direct) {
  UpdateSensors(); // Update the sensor readings
  int tilt = 10;
  float Left_Mid_Reading;
  float Right_Mid_Reading;
  float ultrasonic;

  Left_Mid_Reading = Average[0];
  Right_Mid_Reading = Average[1];
  ultrasonic = Average[4];

  //Robot starts moving forward, will add IR Sensor reading
  forward();
  
  while(ultrasonic >= ForwardDistance){
    UpdateSensors(); // Update the sensor readings
    Left_Mid_Reading = Average[0];
    Right_Mid_Reading = Average[1];
    ultrasonic = Average[4];

    // Rear wheel drive
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);

    if (direct == LEFT) {
      // Front wheel steer
      left_front_motor.writeMicroseconds(1600 - tilt*(Left_Mid_Reading - SideDistance));
      right_front_motor.writeMicroseconds(1400 - tilt*(Left_Mid_Reading - SideDistance)); 
    }

    if (direct == RIGHT) {
      // Front wheel steer
      left_front_motor.writeMicroseconds(1600 - tilt*(-Right_Mid_Reading + SideDistance));
      right_front_motor.writeMicroseconds(1400 - tilt*(-Right_Mid_Reading + SideDistance)); 
    }
    
    }
  stop();
 }

void Shift(DIRECTION direct) {
  // Start motor in correct directions
  if(direct == LEFT) {
    strafe_left();
  } else if(direct == RIGHT) {
    strafe_right();
  }

  delay(2500);
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
  for (int n = 0; n < sizeof(CurrentSensor); n++) {
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
  /*double dist;
  
  LocateCorner();
  MoveToCorner();
  AlignEdge();
  FollowEdge(15, LEFT);
  DIRECTION direct = RIGHT;
  DIRECTION follow_edge;
  while(0) { // distance read in direct direction < 15
    Shift(direct);
    Rotate180();
    dist = FindCloseEdge();
    if(dist > 0) {
      follow_edge = LEFT;
    } else {
      follow_edge = RIGHT;
    }
    FollowEdge(abs(dist), follow_edge);
    
    if(direct = LEFT) {
      direct = RIGHT;
    } else {
      direct = LEFT;
    }
  }
  FollowEdge(15, direct);
  */
  /*FollowEdge(15, 15, RIGHT);*/


  int speedvals[] = {
                      75,
                      100,
                      150,
                      200,
                      250,
                      500,
                      750,
                      1000,
                      1500,
                      -25,
                      -50,
                      -75,
                      -100,
                      -150,
                      -200,
                      -250,
                      -500,
                      -750,
                      -1000,
                      -1500,
};

GYRO_calibrate();



CLRotateDeg(90);
//LocateCorner();

  /*speed_val = speedvals[5];
  BluetoothSerial.println(speed_val);
  cw();
  double total = 0;
  for(int j = 0; j < 100; j++){
    total += GYRO_reading();
    BluetoothSerial.println(GYRO_reading());
    delay(150);
  }
  BluetoothSerial.print("Average: ");
  BluetoothSerial.println(total/100);*/

  delay(1000000);



  /*LocateCorner();
  delay(1000000000);*/
  //disable_motors();
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

  angularVelocity = angularVelocity>rotationThreshold ? angularVelocity : 0;
  //angularVelocity = KalmanFilter(angularVelocity, prevAngularVelocity);

  prevAngularVelocity = angularVelocity; 
  return angularVelocity;
  
}
#endif


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