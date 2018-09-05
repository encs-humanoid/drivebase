/* 
 *  ENCS Humanoid rosserial control code for the mecanum wheel base atmega1280
 *  
 *  This code currently handles the following tasks:
 *      > receives joystick control commands for motor speed control
 *      > calculate mecanum speed co
 
 ntrols for each motor/wheel
 *      > open loop control of motor speed
 *      > read and publish the motor quadrature sensors 
 *      > read and publish the ultrasonic sensor data
 *      > controls the base neopixel lights
 *      
 *  Next steps for this code:     
 *      > add PID control of motor speed
 *      > read and publish 9-DOF IMU data
 *      > add PID control of base space and orientation
 *      > add dynamic neopixel lighting based on obstacle data
 *      
 *  Authors:
 *    Daniel McDonald (2017-2018)
 *    Rodney Radford (2017-2018)
 */

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>

// Global ROS node handle
ros::NodeHandle nh;

/*******************************************************************
 * Ultrasonic sensor declarations
 *******************************************************************/

static const int trigPin1 = 32 ; //(D2) Front Right ultrasonic sensor 1
static const int echoPin1 = 33 ; //(D3) Front Right ultrasonic sensor 1
static const int trigPin2 = 22 ; //(D4) Front Left ultrasonic sensor 2
static const int echoPin2 = 23 ; //(D5) Front Left ultrasonic sensor 2

static const int trigPin3 = 24 ; //(D6) Right Front Corner ultrasonic sensor 3
static const int echoPin3 = 25 ; //(D7) Right Front Corner ultrasonic sensor 3
static const int trigPin4 = 26 ; //(D8) Left Front Corner ultrasonic sensor 4
static const int echoPin4 = 27 ; //(D9) Left Front Corner ultrasonic sensor 4

static const int trigPin5 = 28 ; //(D10) Right Front Corner ultrasonic sensor 5
static const int echoPin5 = 29 ; //(D11) Right Front Corner ultrasonic sensor 5
static const int trigPin6 = 30 ; //(D12) Left Front Corner ultrasonic sensor 6
static const int echoPin6 = 31 ; //(D13) Left Front Corner ultrasonic sensor 6

#define NUM_ULTRASONICS  6
static const int trigPins[] = { trigPin1, trigPin2, trigPin3, trigPin4, trigPin5, trigPin6 };
static const int echoPins[] = { echoPin1, echoPin2, echoPin3, echoPin4, echoPin5, echoPin6 };

// current ultrasonic sensor to read (advance forward once on each read)
int currentSensor = 0;

// How often to run the loop - this time should be longer than
//   the time to read one ultrasonic sensor and process the ros callbacks
//   and publications
#define LOOP_INTERVAL_TIME   50 // 20ms for ultrasonic read
int nextLoopInterval = 0;

/*******************************************************************
 * Ultrasonic sensor routines
 *******************************************************************/

int getDistance(int trigPin, int echoPin)
{
  // Clears the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigger pin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the round trip time for the echo time microseconds
  long duration = pulseIn(echoPin, HIGH, 10000);
  
  // Convert the round trip echo time to distance in centimeters
  int distance = duration*0.034/2;
  return distance;
}


/*******************************************************************
 * Motor control declarations
 *******************************************************************/

/*
 * Motor control ESC values for neutral and max forward and reverse
 * Note that 90 is neutral, 0 is max reverse and 179 is max forward
 * These values limit that range as the extreme values are just TOO FAST
 */
#define MOTOR_NEUTRAL    92
#define MOTOR_MAX_SPEED  30  // 10-30
#define MOTOR_MAX_FWD  (MOTOR_NEUTRAL + MOTOR_MAX_SPEED)
#define MOTOR_MAX_REV  (MOTOR_NEUTRAL - MOTOR_MAX_SPEED)

/*
 * Map the motor numbers (on the Ken frame) to Arduino pin numbers
 */
#define MOTOR1   18   // Front right
#define MOTOR2   19   // Back left
#define MOTOR3   20   // Back right
#define MOTOR4   21   // Front left

Servo   frontRight;   // Motor 1
Servo   backLeft;     // Motor 2
Servo   backRight;    // Motor 3
Servo   frontLeft;    // Motor 4

/*******************************************************************
 * Motor control subroutines
 *******************************************************************/

/*
 * Initialize the motor servo controls
 */
void MotorSetup() 
{
    /* Initialize the four servo functions for the four motors */
    frontRight.attach(MOTOR1);
    backLeft.attach(MOTOR2);
    backRight.attach(MOTOR3);   
    frontLeft.attach(MOTOR4);
    
    /* Turn the motors off */
    MotorDrive(0,0,0,0);
}


/* 
 * Map a joystick speed of [-128..+127] to a motor speed, handling both
 *    overflow (outside of range [-127..+127], and create a deadstick space
 *    in the middle to avoid motor creep at middle joystick position
 */
static int MotorMapSpeed(int in)
{  
  if (in > 127) {
     in = 127;
  } else if ((in >= -5) && (in <= 5)) {
     in = 0;
  } else if (in < -127) {
     in = -127;
  }
  
  return map(in,-127,127,MOTOR_MAX_REV,MOTOR_MAX_FWD);
}

/*
 * Send speeds to the motor. 
 *
 * The ROS motor speeds are mapped to be between [-128, +127]
 *
 * Note that the forward/reverse is as respect to the motor, and not it's
 *    orientation within the chassis - for example, those on the left and 
 *    those on the right have a different view of forward/reverse. 
 */
void MotorDrive(int fl, int fr, int bl, int br)
{
  /* Map the ROS speeds to the min/max motor speeds */
  int fr2 = MotorMapSpeed(-fr);
  int bl2 = MotorMapSpeed(bl);
  int br2 = MotorMapSpeed(-br);
  int fl2 = MotorMapSpeed(fl);

//  nh.loginfo("MotorDrive");
  
  frontRight.write(fr2);
  backLeft.write(bl2);
  backRight.write(br2);
  frontLeft.write(fl2);
}


/*******************************************************************
 * ROS joystick control callback
 *******************************************************************/

// Global values that are written to on each ROS joystick twist message
int drive=0, strafe=0, theta=0;

/*
 * ROS callback function that is invoked on each ROS joystick message
 */
void on_twist(const geometry_msgs::Twist& twist_msg)
{
   // Map double [-1.0..+1.0] to integral [-127..128] and save to globals
   drive  = (int) (127.0 * twist_msg.linear.y);
   strafe = (int) (127.0 * twist_msg.linear.x);
   theta  = (int) (127.0 * twist_msg.angular.z);
}

/*
 * Convert the drive (front/back), strafe (left/right) and theta (twist)
 *    joystick values into individual motor movement values and write them
 *    out to the motor speed controllers
 */
void MecanumDrive(int drive, int strafe, int theta)
{
   int front_left  = drive - strafe + theta;
   int back_left   = drive - strafe - theta;
   int front_right = drive + strafe - theta;
   int back_right  = drive + strafe + theta;

  MotorDrive(front_left,front_right,back_right,back_left); 
}



/*******************************************************************
 * Quadrature encoder interrupt routine
 *
 * This quadrature encoder interrupt handler is based heavily
 *   on an the implementation in the Encoder library at:
 *
 *     https://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * First an overview of the two lines in a quadrature encoder:
 *               _____       _____
 *   Pin1:  ____|     |_____|     |____
 *            _____       _____       _  
 *   Pin2:  _|     |_____|     |_____|
 *
 *        <--- negative     positive --->
 *
 * By examining the current two pins and the two previous
 *    pins, we can determine the position and amount of
 *    motion (assume we can respond fast enough to only 
 *    lose at most 1 transition).
 *
 *   new    new    old    old
 *   pin2   pin1   pin2   pin1   Result
 *   ----   ----   ----   ----   ------
 *    0      0      0      0     no movement
 *    0      0      0      1     +1
 *    0      0      1      0     -1
 *    0      0      1      1     +2
 *    0      1      0      0     -1
 *    0      1      0      1     no movement
 *    0      1      1      0     -2
 *    0      1      1      1     +1
 *    1      0      0      0     +1
 *    1      0      0      1     -2
 *    1      0      1      0     no movement
 *    1      0      1      1     -1
 *    1      1      0      0     +2
 *    1      1      0      1     -1
 *    1      1      1      0     +1
 *    1      1      1      1     no movement
 *   ----   ----   ----   ----   ------
 */

// state change based on table above
static const int stateChange[16] = 
{  
   0, +1, -1, +2, -1,  0, -2, +1, 
  +1, -2,  0, -1, +2, -1, +1,  0 
};

// Phase A and phase B pins for each quadrature
// Note: quadPins is arranged as {{4A,4B},{3A,3B},{2A,2B},{1A,1B}} with interrupts
//       on phase A, and non-interrupts on phase B
#define NUM_QUADRATURES  4
static const int quadPins[NUM_QUADRATURES][2] = {{10,4}, {11,5}, {12,6}, {13,7}};
static const int quadPinShift[NUM_QUADRATURES] = {0, 2, 4, 6};
volatile int quadValues[NUM_QUADRATURES];
volatile int previousQuads = 0;

static void quadratureInterrupt(void) 
{
  // Read the current state of the quadrature pins into a single 8 bit value
  //    Bits are arranged as MSB quad3A,quad3B down to LSB quad0A,quad0B
  int currentQuads = 0x0;
  for (int i = 0; i < NUM_QUADRATURES; i++) {
     currentQuads |= digitalRead(quadPins[i][0]) << (quadPinShift[i]+1);
     currentQuads |= digitalRead(quadPins[i][1]) << quadPinShift[i];
  }
  
  // Loop through each of the quadratures, updating the current position values
  for (int i = 0; i < NUM_QUADRATURES; i++) {

     // Build up the 4bit state of current and past pin values
     int quadState = ((previousQuads >> quadPinShift[i]) & 0x03) |
                     (((currentQuads >> quadPinShift[i]) & 0x03) << 2);
                     
     // Based on 4-bit state value, update encoder value
     quadValues[i] += stateChange[quadState];
  }

  // Save our current quadrature pin values for the next interrupt
  previousQuads = currentQuads;
}


/*******************************************************************
 * Setup function - called once at start of program
 *******************************************************************/

// Structure and callback for ultrasound sensor ROS publication
char frameid[32];
sensor_msgs::Range rangeData;
ros::Publisher range_pub( "/ultrasound", &rangeData);

// Structure and callback for quadrathre ROS publication
std_msgs::Int16MultiArray quadArray;
ros::Publisher quad_pub("/quadrature", &quadArray);

// Subscriber callback for joystick movements
ros::Subscriber<geometry_msgs::Twist> twist_sub("/cmd_vel", &on_twist);

void setup()
{
  // Initialize the USB serial port and set the port speed
  Serial.begin(57600);

  // Create the ROS node handle instance
  nh.initNode();

  // Register the ROS node handle callbacks 
  nh.advertise(range_pub);
//  nh.advertise(quad_pub);
  nh.subscribe(twist_sub);

  // Initialize the motor controllers and stopped motor speed
  MotorSetup();

  // Initialize the range message (used by range_pub callback)
  rangeData.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeData.field_of_view = 0.1;  // fake
  rangeData.min_range = 0.0;
  rangeData.max_range = 6.47;
  
  // Set up the ultrasonic pins - ouput for trigger, input for echo
  for (int i = 0; i < NUM_ULTRASONICS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}


/*******************************************************************
 * Loop function - called continuously while the Arduino is on
 *******************************************************************/

void loop()
{
  // Since the time for reading the ultrasonic is variable due to short
  //   or long echo times, this logic tries to keep the interval fairly
  //   consistent from reading to reading. This is temporary until the
  //   ultrasonic code is rewritten to be interrupt driven
//  int now = millis();
//  if (now < nextLoopInterval) {
//     return;
//  }
//  nextLoopInterval = now + LOOP_INTERVAL_TIME;
  
  // Read just one distance sensor now
  int distance = getDistance(trigPins[currentSensor], echoPins[currentSensor]);

  // Convert distance to meters and update the rangeData data
  rangeData.range = distance / 100.0f;  // convert cm to meters
  rangeData.header.stamp = nh.now();
  sprintf(frameid, "/ultrasound_%d", currentSensor + 1);
  rangeData.header.frame_id = frameid;

  // Advance forward to get ready for the next sensor read
  currentSensor = (currentSensor+1) % NUM_ULTRASONICS;     

  // Publish the updated ultrasonic distance values
  range_pub.publish(&rangeData);

  // Publish quadrature encoder values
  // https://answers.ros.org/question/223565/array-data-from-arduino/
//  for (int i = 0; i < NUM_QUADRATURES; i++) {
//     quadArray.data[i] = quadValues[i];
//  }
//  quad_pub.publish(&quadArray);
    
  // spinonce gives permission for ROS to invoke our callbacks since 
  //   we do not have preemptive tasking so we need to fill in any data
  //   that is a publish (like range_pub) before spinonce, and do any
  //   actions that consume the data (like adjust motor speed) after spinonce
  nh.spinOnce();

  // Now that the callbacks have been called, we can update the drive motors
  MecanumDrive(drive,strafe,theta);
  
  delay(30);
}

