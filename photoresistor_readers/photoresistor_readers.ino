/*
   CCRI Pick and Place Cabinet Object Detection
   Utilizes photoresistors to indicate whether or not an object is covering one of the fiducial markers.
   External lighting may need to be added to cabinet to ensure proper operation (LED strips, etc.) Publishes 
   data over ROS Serial and data can be stored locally on the PC. Requires you to run the command:
   rosrun roserial_python serial_node.py /dev/ttyACM0   //where /dev/ttyACM0 is the port the uController is on
   Tested on Teensy 3.6 and 3.2 with teensyduino, results may vary on other boards.Tested on Ubuntu 16.04 with
   ROS Kinetic.
*/
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#define NUM_SENSORS 5

//Set Analog Pin Definitions for your board here
//used for the target teensy 3.6 device
const int adc_pins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

//Set ADC Precision here
//8 default, 12 recommended to avoid noise, 16 max in hardware but will pick up noise.
const int anlg_rd_precision = 12;

ros::NodeHandle nh;

std_msgs::UInt32MultiArray light_msg;
ros::Publisher lights("lights2", &light_msg);


void setup()
{
  nh.initNode();
  light_msg.data = (unsigned long*)malloc(sizeof(unsigned long) * NUM_SENSORS);
  light_msg.data_length = NUM_SENSORS;


  analogReadResolution(anlg_rd_precision);
  for (int i = 0; i < NUM_SENSORS; i++) {
    light_msg.data[i] = 0;
  }

  nh.advertise(lights);
}



void update_msg(std_msgs::UInt32MultiArray& light_msg) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    light_msg.data[i] = analogRead(adc_pins[i]);
  }
}

void loop()
{
  lights.publish(&light_msg);

  update_msg(light_msg);
  nh.spinOnce();
  delay(20);
}
