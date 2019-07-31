#include <Adafruit_NeoPixel.h>

/* 
 *  ENCS Humanoid rosserial arduino code for distance sensor lighting display
 *  
 *  This code currently handles the following tasks:
 *      > receives ultrasound messages
 *      > computes the color to display on the light strip corresponding to 
 *        the position of the sensor
 *      > sets the light color
 *      > pulses the backend lights which don't have sensors connected
 *
 *  Authors:
 *    Daniel McDonald (2017-2019)
 *    Matthew Kesselring (2018)
 *    
 *  NOTES:
 *    To upload to the DaleKEN Arduino Nano set
 *      board = Arduino Nano
 *      port = /dev/ttyUSB0
 *      processor = ATmega328P (Old Bootloader)
 *      programmer = AVRISP mkII
 */
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

//ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 128> nh;
ros::NodeHandle nh;

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
  
  char *ret = a;
  long integer = (long)f;
  itoa(integer, a, 10);
  while (*a != '\0')
    a++;
  *a++ = '.';
  long decimal = abs((long)((f - integer) * p[precision]));
  itoa(decimal, a, 10);
  return ret;
}


/*******************************************************************
 * Neopixel declarations
 *******************************************************************/
#define NUMPIXELS      30
#define NEOPIXEL_PIN   2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_BRG + NEO_KHZ800);

int posn;


/*******************************************************************
 * Ultrasound range declarations
 *******************************************************************/

#define NUM_SENSORS    6
//#define NUM_SENSORS 12

#define START_BACK_PIXEL 7
#define END_BACK_PIXEL 22
#define MIN_RANGE 0.0f
#define MAX_RANGE 1.0f
#define MIN_STEPS 25
#define MAX_STEPS 100

float ranges[NUM_SENSORS];
// pixel numbers which are fractions indicate that the sensor is located
// between two pixels.  The code should light up both the floor(pixel) and
// the ceil(pixel).
float sensor_pixels[] = {1.5f, 28.5f, 3.5f, 26.5f, 5.5f, 24.5f};
//float sensor_pixels[] = {1.5f, 28.5f, 3.5f, 26.5f, 5.5f, 24.5f, 9.5f, 20.5f, 11.5f, 18.5f, 13.5f, 16.5f}; // 15, 7, 8, 23, 22 are not associated with sensors
int dir = 1;

/*******************************************************************
 * ROS subscriber callbacks
 *******************************************************************/
long msgCount = 0;

/*
void on_range(const sensor_msgs::Range& range_msg)
{
  int sensor;
  sscanf(range_msg.header.frame_id, "/ultrasound_%d", &sensor);
  ranges[sensor - 1] = range_msg.range;
  ++msgCount;
}
*/

void on_range_str(const std_msgs::String& range_msg)
{
  // range values are expected to be integers in millimeter units
  int mm[NUM_SENSORS];
  sscanf(range_msg.data, "%d %d %d %d %d %d", &mm[0], &mm[1], &mm[2], &mm[3], &mm[4], &mm[5]);
  for (int i = 0; i < NUM_SENSORS; ++i) {
    ranges[i] = mm[i] / 1000.0f;  // convert millimeters to meters
  }
  ++msgCount;
}

//ros::Subscriber<sensor_msgs::Range> range_sub("/ultrasound", &on_range);
ros::Subscriber<std_msgs::String> range_sub("/sonar_array_10hz", &on_range_str);


/*******************************************************************
 * setup
 *******************************************************************/

void setup()
{
  // Initialize the USB serial port and set the port speed
  Serial.begin(57600);
  
  // Create the ROS node handle instance
  nh.initNode();
  
  // Initialize the Neopixels
  pixels.begin();
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0xFF, 0x00, 0x00));
  }
  pixels.show();
  
  posn = START_BACK_PIXEL;
  
  // Initialize the range array
  for (int i = 0; i < NUM_SENSORS; i++) {
    ranges[i] = 0.0f;
  }
  
  // Register the ROS node handle callback
  nh.subscribe(range_sub);
}

/*******************************************************************
 * loop
 *******************************************************************/

int get_color(float range)
{
  int color = min(0xFF, max(0x00, (int) map(range*100, MIN_RANGE*100, MAX_RANGE*100, 0xFF, 0x00)));
  return color;
}

void set_pixel_color(int pixel, int color)
{
  int red, green, blue;
  int maxRed = 0xFF;
  int maxGreen = 0x80;
  int maxBlue = 0x60;
  int greenRange = 0;
  int midRange = 100;
  int redRange = 200;
  
  if (color < midRange) {
    red = 0;
    green = min(maxGreen, max(0x00, map(color, greenRange, midRange, maxGreen, 0x00)));
    blue = min(maxBlue, max(0x00, map(color, greenRange, midRange, 0x00, maxBlue)));
  }
  else {
   red = min(maxRed, max(0x00, map(color, midRange, redRange, 0x00, maxRed)));
   green = 0;
   blue = min(maxBlue, max(0x00, map(color, midRange, redRange, maxBlue, 0x00)));
  }
  
  pixels.setPixelColor(pixel, pixels.Color(red, green, blue));
}

void loop()
{
  char buffer[32];
  sprintf(buffer, "msgs: %d", msgCount);
  nh.loginfo(buffer);
  nh.spinOnce();
  
  pixels.clear();
  int colors[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    colors[i] = get_color(ranges[i]);
    int pixel1 = (int) floor(sensor_pixels[i]);
    int pixel2 = (int) ceil(sensor_pixels[i]);
    //sprintf(buffer, "pixels: %d, %d", pixel1, pixel2);
    //nh.loginfo(buffer);
    set_pixel_color(pixel1, colors[i]);
    set_pixel_color(pixel2, colors[i]);
  }
  
  for (int i = START_BACK_PIXEL; i <= END_BACK_PIXEL + 1; i++) {
    float frac = posn / ((float) MAX_STEPS);
    pixels.setPixelColor(i, pixels.Color((int) (frac*0xFF), (int) (frac*0x60), 0x00)); // YELLOW
  }

  pixels.show();
  
  posn += dir;
  
  if (posn > MAX_STEPS) {
    posn = MAX_STEPS - 1;
    dir = -1;
  }
  else if (posn < MIN_STEPS) {
    posn = MIN_STEPS + 1;
    dir = 1;
  }
  
  delay(25);
}
