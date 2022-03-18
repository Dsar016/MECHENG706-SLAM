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
const int MID_RANGE_LEFT_PIN = A12;
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
const int Mid_Left_Exponent = 1 / 1.001;
const int Mid_Left_Power = pow(2362.6, Mid_Left_Exponent);
const int Long_Left_Exponent = 1 / 0.901;
const int Long_Left_Power = pow(3720.4, Long_Left_Exponent);
const int Mid_Right_Exponent = 1 / 0.695;
const int Mid_Right_Power = pow(2148.8, Mid_Right_Exponent);

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
  OCR2A = 250;
  TCCR2A |= (1 << WGM11); // CTC mode
  TCCR2B |= (1 << CS01) | (1<<CS00); // no prescaler
  TIMSK2 |= (1 << OCIE1A);
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
  };
}

///////////////////// PROTOTYPE 1 FUNCTIONS ///////////////////////

int speedX = 0, speedY = 0, rSpeedZ = 0; // m/s and rad/s
const float L1 = 0.09, L2 = 0.09; //dimensions in m

int msCount2 = 0; //millisecond count on timer 2

/**
 * updates the x and y speeds of the robot according to the global speed variables
 */
void DriveXYZ() {
  left_front_motor.writeMicroseconds(1500 + speedX + speedY - (L1+L2)*rSpeedZ);
  right_front_motor.writeMicroseconds(1500 + speedX - speedY + (L1+L2)*rSpeedZ);
  left_rear_motor.writeMicroseconds(1500 + speedX - speedY  - (L1+L2)*rSpeedZ);
  right_rear_motor.writeMicroseconds(1500 + speedX + speedY + (L1+L2)*rSpeedZ);
}

void RotateDeg(int degrees = 0){
  float theta = 0;
  rSpeedZ = 10;
 
  while(theta < degrees){
    DriveXYZ();
    theta = rSpeedZ * (PI/180) * (1000*msCount2);
  }

  rSpeedZ = 0;
}

ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;
  msCount2++;
}

void LocateCorner(void) {
  
}

void MoveToCorner(void) {
  
}

void AlignEdge(void) {
  
}

void FollowEdge(int distance, DIRECTION direct) {
  
}

void Shift(DIRECTION direct) {
  
}

void Rotate180(void) {
  
}

double FindCloseEdge(void) {
  
}

////////////////// SENSOR FUNCTIONS /////////////////////////////////


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

  speedX = 10;
  DriveXYZ();
  /*static unsigned long previous_millis;

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();

  #ifndef NO_READ_GYRO
      GYRO_reading();
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
  double dist;
  
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

  FollowEdge(15, direct);*/
  // END OF PROTOTYPE 1 ///////////////

  return RUNNING;
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
void HC_SR04_range()
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
      SerialCom->println("HC-SR04: NOT found");
      return;
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
      SerialCom->println("HC-SR04: Out of range");
      return;
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
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

#ifndef NO_READ_IR
void IR_reading(IR_SENSOR sensor)
{
  SerialCom->print("IR Sensor:");
  switch (sensor)
  {
    case LEFT_MID: 
      SerialCom->print("Mid Left IR Sensor: ");
      SerialCom->print((Mid_Left_Power) / (pow(analogRead(MID_RANGE_LEFT_PIN),Mid_Left_Exponent))); //MID_RANGE_LEFT_PIN
      SerialCom->println(" Cm");
      break;
    case LEFT_LONG:
      SerialCom->print("Long Left IR Sensor: ");
      SerialCom->print(Long_Left_Power) / (pow(analogRead(LONG_RANGE_LEFT_PIN),Long_Left_Exponent)); //LONG_RANGE_LEFT_PIN
      SerialCom->println(" Cm");
      break;
    case RIGHT_LONG:
      SerialCom->print("Long Right IR Sensor: ");
      //Must be calculated in the switch statement, as the analog read is right in the middle of the equation, so its always changing
      SerialCom->print(-(log(analogRead(LONG_RANGE_RIGHT_PIN) / 379.76)/0.056)); //LONG_RANGE_RIGHT_PIN
      SerialCom->println(" Cm");
      break;
     case RIGHT_MID:
      SerialCom->print("Mid Right IR Sensor: ");
      SerialCom->print((Mid_Right_Power) / (pow(analogRead(MID_RANGE_RIGHT_PIN),Mid_Right_Exponent))); //MID_RANGE_RIGHT_PIN
      SerialCom->println(" Cm");
      break;
  }
}
#endif

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(GYRO_PIN));
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
