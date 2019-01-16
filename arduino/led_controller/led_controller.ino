#include <Adafruit_NeoPixel.h>

/* 
 * rosserial Ultrasound
 * 

 */
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
char buffer[128];
char buffer2[32];

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}


/*******************************************************************
 * Neopixel declarations
 *******************************************************************/
#define NEOPIXEL_PIN   2
#define NUMPIXELS      30

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_BRG + NEO_KHZ800);

int posn;
int pixel_color;


/*******************************************************************
 * Ultrasound range declarations
 *******************************************************************/

#define NUM_SENSORS    6

//#define NUM_SENSORS 12

float ranges[NUM_SENSORS];
float min_range = 0;
float max_range = 1;
// pixel numbers which are fractions indicate that the sensor is located
// between two pixels.  The code should light up both the floor(pixel) and
// the ceil(pixel).
float sensor_pixels[] = {1.5f, 28.5f, 3.5f, 26.5f, 5.5f, 24.5f};
//float sensor_pixels[] = {1.5f, 28.5f, 3.5f, 26.5f, 5.5f, 24.5f, 9.5f, 20.5f, 11.5f, 18.5f, 13.5f, 16.5f}; // 15, 7, 8, 23, 22 are not associated with sensors
int start_back_pixel = 7;
int end_back_pixel = 22;
int dir = 1;
int min_steps = 25;
int max_steps = 100;

/*******************************************************************
 * ROS subscriber callbacks
 *******************************************************************/
void on_range(const sensor_msgs::Range& range_msg)
{
  // TODO
  int sensor;
  sscanf(range_msg.header.frame_id, "/ultrasound_%d", &sensor);
  ranges[sensor - 1] = range_msg.range;
  //nh.loginfo(range_msg.header.frame_id);
  //min_range = range_msg.min_range;
  //max_range = range_msg.max_range;
}

ros::Subscriber<sensor_msgs::Range> range_sub("/ultrasound", &on_range);


/*******************************************************************
 * setup
 *******************************************************************/

void setup()
{
  Serial.begin(57600); //  (9600) Starts the serial communication
  
  nh.initNode();
  
  // Simple test of the Neopixels (half blue to all pixels)
  pixels.begin();
  pixels.clear();
  for (int i=0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0xFF,0x00,0x00));
  }
  pixels.show();
  pixel_color=0;
  posn = start_back_pixel;
  
  // Initialize the range array
  for (int i = 0; i < NUM_SENSORS; i++) {
    ranges[i] = 0.0f;
  }
  
  // Register ROS subscriber
  nh.subscribe(range_sub);
}

/*******************************************************************
 * loop
 *******************************************************************/

int get_color(float range)
{
  int color = min(0xFF, max(0x00, (int) map(range*100, min_range*100, max_range*100, 0xFF, 0x00)));
  return color;
}

void set_pixel_color(int pixel, int color)
{
  /*if (color < 90) {
    pixels.setPixelColor(pixel, pixels.Color(0x00,0x80,0x00)); // GREEN
  }
  else if (color < 150) {
    pixels.setPixelColor(pixel, pixels.Color(0x00,0x00,0x80)); // BLUE
  }
  else {
    pixels.setPixelColor(pixel, pixels.Color(0x80,0x00,0x00)); // RED
  }*/
  int red, green, blue;
  int maxRed = 0xFF;
  int maxGreen = 0x80;
  int maxBlue = 0x60;
  int greenRange = 0;
  int midRange = 100;
  int redRange = 200;
  if(color < midRange){
    red = 0;
    green = min(maxGreen,max(0x00,map(color,greenRange,midRange,maxGreen,0x00)));
    blue = min(maxBlue,max(0x00,map(color,greenRange,midRange,0x00,maxBlue)));
  }
  else{
   red = min(maxRed,max(0x00,map(color,midRange,redRange,0x00,maxRed)));
   green = 0;
   blue = min(maxBlue,max(0x00,map(color,midRange,redRange,maxBlue,0x00)));
  }
  pixels.setPixelColor(pixel, pixels.Color(red,green,blue));
}

void loop()
{
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
  
  for (int i = start_back_pixel; i <= end_back_pixel + 1; i++) {
    float frac = posn / ((float) max_steps);
    pixels.setPixelColor(i, pixels.Color((int) (frac*0xFF), (int) (frac*0x60), 0x00)); // YELLOW
  }


  pixels.show();
  
  posn += dir;
  
  if (posn > max_steps) {
    posn = max_steps - 1;
    dir = -1;
  }
  else if (posn < min_steps) {
    posn = min_steps + 1;
    dir = 1;
  }

  //sprintf(buffer, "%d, %d, %d, %d, %d, %d", colors[5], colors[3], colors[1], colors[0], colors[2], colors[4]);
  //nh.loginfo(buffer);
  
  delay(25);
}
