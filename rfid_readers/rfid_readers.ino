/*
 * CCRI Pick and Place RFID Shelf Object Detection
 * The shelf uses multiple RFID readers to detect whether objects are placed on the various fiducial markers on the shelf and identifies each object placed on the markers.
 * The object data is sent over ROS Serial to a master device which processes the object data.  Requires you to run the command:
 * rosrun rosserial_python serial_node.py /dev/ttyACM0   //where /dev/ttyACM0 is the port the uController is on.  The data array will contain either a 0 for no object placed
 * or the ID of the RFID tag that is placed on top of the marker.
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>
#include <MFRC522.h>
#include <SPI.h>


#define NUM_RFIDS 5

#define RESET_PIN 5


//Set SS Pin Definitions for your board here
const int rfid_ss_pins[NUM_RFIDS] = {0, 1, 2, 3, 4};

ros::NodeHandle nh;

std_msgs::ByteMultiArray rfid_msg;
ros::Publisher rfid_publisher("rfid2", &rfid_msg);

MFRC522 mfrc522[NUM_RFIDS];

void setup()
{
  for(uint8_t i = 0; i < NUM_RFIDS; i++)
  {
    pinMode(rfid_ss_pins[i], OUTPUT);
  }
  
  nh.initNode();
  //Allocate space for a multidimensional array with a width of the # of sensors and a height of 10 bytes
  rfid_msg.data = (signed char*)malloc(sizeof(signed char) * NUM_RFIDS * 10);
  rfid_msg.data_length = NUM_RFIDS * 10;
  //rfid_msg.layout.dim[2];
  //rfid_msg.layout.dim_length = 2;
  //rfid_msg.layout.dim[0].size = NUM_RFIDS;
  //rfid_msg.layout.dim[0].stride = 10 * NUM_RFIDS;
  //rfid_msg.layout.dim[1].size = 10;
  //rfid_msg.layout.dim[1].stride = 10;
  

  SPI.begin();        // Init SPI bus

  for (uint8_t reader = 0; reader < NUM_RFIDS; reader++) {
    mfrc522[reader].PCD_Init(rfid_ss_pins[reader], RESET_PIN); // Init each MFRC522 card
  }

  //Initialize data array
  for (int i = 0; i < NUM_RFIDS * 10; i++) {
    rfid_msg.data[i] = 0;
  }

  nh.advertise(rfid_publisher);
}


void update_msg(std_msgs::ByteMultiArray& rfid_msg)
{
  for (uint8_t reader = 0; reader < NUM_RFIDS; reader++)
  {
    // Look for new cards
    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial())
    {
      for(uint8_t i = 0; i < mfrc522[reader].uid.size; i++)
      {
        rfid_msg.data[(10 * reader) + i] = mfrc522[reader].uid.uidByte[i];
      }

      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();
    }
    else
    {
      for(uint8_t i = 0; i < mfrc522[reader].uid.size; i++)
      {
        rfid_msg.data[(10 * reader) + i] = 0;
      }
    }
  }
}

void loop()
{
  rfid_publisher.publish(&rfid_msg);

  update_msg(rfid_msg);
  nh.spinOnce();
  delay(20);
}
