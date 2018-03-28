/* 
 * rosserial Ultrasound
 * 

 */
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>

ros::NodeHandle  nh;

/*******************************************************************
 * Ultrasonic sensor declarations
 *******************************************************************/

#if 0
int trigPin1 = 20 ; //(D20) Front Right ultrasonic sensor 1
int echoPin1 = 21 ; //(D21) Front Right ultrasonic sensor 1
int trigPin2 = 22 ; //(D22) Front Left ultrasonic sensor 2
int echoPin2 = 23 ; //(D23) Front Left ultrasonic sensor 2

int trigPin3 = 24 ; //(D24) Right Front Corner ultrasonic sensor 3
int echoPin3 = 25 ; //(D25) Right Front Corner ultrasonic sensor 3
int trigPin4 = 26 ; //(D26) Left Front Corner ultrasonic sensor 4
int echoPin4 = 27 ; //(D27) Left Front Corner ultrasonic sensor 4

int trigPin5 = 28 ; //(D28) Right Front Corner ultrasonic sensor 5
int echoPin5 = 29 ; //(D29) Right Front Corner ultrasonic sensor 5
int trigPin6 = 30 ; //(D30) Left Front Corner ultrasonic sensor 6
int echoPin6 = 31 ; //(D31) Left Front Corner ultrasonic sensor 6

int numSensors = 6; // Number of ultrasonic sensors mounted on the robot
int trigPins[] = { trigPin1, trigPin2, trigPin3, trigPin4, trigPin5, trigPin6 };
int echoPins[] = { echoPin1, echoPin2, echoPin3, echoPin4, echoPin5, echoPin6 };
#else
int trigPin1 = 2 ; //(D20) Front Right ultrasonic sensor 1
int echoPin1 = 3 ; //(D21) Front Right ultrasonic sensor 1
int trigPin2 = 4 ; //(D22) Front Left ultrasonic sensor 2
int echoPin2 = 5 ; //(D23) Front Left ultrasonic sensor 2

int trigPin3 = 6 ; //(D24) Right Front Corner ultrasonic sensor 3
int echoPin3 = 7 ; //(D25) Right Front Corner ultrasonic sensor 3
int trigPin4 = 8 ; //(D26) Left Front Corner ultrasonic sensor 4
int echoPin4 = 9 ; //(D27) Left Front Corner ultrasonic sensor 4

int trigPin5 = 10 ; //(D28) Right Front Corner ultrasonic sensor 5
int echoPin5 = 11 ; //(D29) Right Front Corner ultrasonic sensor 5
int trigPin6 = 22 ; //(D30) Left Front Corner ultrasonic sensor 6
int echoPin6 = 23 ; //(D31) Left Front Corner ultrasonic sensor 6

int numSensors = 6; // Number of ultrasonic sensors mounted on the robot
int trigPins[] = { trigPin1, trigPin2, trigPin3, trigPin4, trigPin5, trigPin6 };
int echoPins[] = { echoPin1, echoPin2, echoPin3, echoPin4, echoPin5, echoPin6 };
#endif

int currentSensor = 0;
int sweepTime = numSensors * 40;  // Number of milliseconds to sweep all sensors
char frameid[32];

/*******************************************************************
 * Ultrasonic sensor routines
 *******************************************************************/

int getDistance(int trigPin, int echoPin)
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microsecondst
  long duration = pulseIn(echoPin, HIGH, 10000);
  
  // Calculating the distance
  int distance = duration*0.034/2;
  return distance;
}


/*******************************************************************
 * Motor control declarations
 *******************************************************************/

/*
 * Map the motor numbers (on the Ken frame) to Arduino pin numbers
 */
#define MOTOR1   18  
#define MOTOR2   19
#define MOTOR3   20
#define MOTOR4   21

/*
 * Motor control ESC values for neutral and max forward and reverse
 * Note that 90 is neutral, 0 is max reverse and 179 is max forward
 * These values limit that range as the extreme values are just TOO FAST
 */
#define MOTOR_NEUTRAL    92
#define MOTOR_MAX_SPEED  20  // 10-30
#define MOTOR_MAX_FWD  (MOTOR_NEUTRAL + MOTOR_MAX_SPEED)
#define MOTOR_MAX_REV  (MOTOR_NEUTRAL - MOTOR_MAX_SPEED)

Servo   frontLeft;
Servo   frontRight;
Servo   backLeft;
Servo   backRight;

/*
 * Motor setup and motor control routines
 */
void MotorSetup(void);
void MotorSpeed(int fl, int fr, int bl, int br);

/*******************************************************************
 * Motor control subroutines
 *******************************************************************/

/*
 * Initialize the motor servo controls
 */
void MotorSetup() 
{
    /* Initialize the four servo functions for the four motors */
    frontLeft.attach(MOTOR4);
    frontRight.attach(MOTOR1);
    backLeft.attach(MOTOR2);
    backRight.attach(MOTOR3);   
    
    /* Turn the motors off */
    MotorDrive(0,0,0,0);
}


/* 
 * Map a joystick speed of [-128..+127] to a motor speed
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
 *    those on the right have a different view of forward/reverse. This
 *    should be handled in the ROS level above this code
 */
void MotorDrive(int fl, int fr, int bl, int br)
{
  /* Map the ROS speeds to the min/max motor speeds */
  int fl2 = MotorMapSpeed(fl);
  int fr2 = MotorMapSpeed(-fr);
  int bl2 = MotorMapSpeed(bl);
  int br2 = MotorMapSpeed(-br);

  nh.loginfo("MotorDrive");
  
  frontLeft.write(fl2);
  frontRight.write(fr2);
  backLeft.write(bl2);
  backRight.write(br2);
}

/*******************************************************************
 * ROS joystick control callback
 *******************************************************************/

int drive=0, strafe=0, theta=0;

void on_twist(const geometry_msgs::Twist& twist_msg)
{
   // Map double [-1.0..+1.0] to integral [-127..128]
   drive  = (int) (127.0 * twist_msg.linear.y);
   strafe = (int) (127.0 * twist_msg.linear.x);
   theta  = (int) (127.0 * twist_msg.angular.z);
}

void MecanumDrive(int drive, int strafe, int theta)
{
#if 0
  int front_left = drive;
  int front_right = strafe;
  int back_right = theta;
  int back_left = -theta;   
#else
   int front_left  = drive + strafe + theta;
   int back_left   = drive + strafe - theta;
   int front_right = drive - strafe - theta;
   int back_right  = drive - strafe + theta;
#endif

  MotorDrive(front_left,front_right,back_right,back_left); 
}


/*******************************************************************
 * setup
 *******************************************************************/

ros::Subscriber<geometry_msgs::Twist> twist_sub("/cmd_vel", &on_twist);
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

void setup()
{
  Serial.begin(57600); //  (9600) Starts the serial communication
 
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(twist_sub);
  
  MotorSetup();
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  
  for (int i = 0; i < numSensors; ++i) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

/*******************************************************************
 * loop
 *******************************************************************/


void loop()
{
  int distance = getDistance(trigPins[currentSensor], echoPins[currentSensor]);

//   MotorDrive(127,127,-127,-127);
 
//  nh.loginfo("loop");

  // publish the distance value
  range_msg.range = distance / 100.0f;  // convert cm to m
  range_msg.header.stamp = nh.now();
  sprintf(frameid, "/ultrasound_%d", currentSensor + 1);
  range_msg.header.frame_id = frameid;
  pub_range.publish(&range_msg);
  
  nh.spinOnce();
  
  MecanumDrive(drive,strafe,theta);
  
  delay(sweepTime/numSensors);
  ++currentSensor;
  if (currentSensor >= numSensors) {
    currentSensor = 0;
  }
}



