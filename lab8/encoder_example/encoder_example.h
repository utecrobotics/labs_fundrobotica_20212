#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define ENCODER_PUBLISH_FREQUENCY        500   //hz

#define encaR  19   // Yellow cable
#define encbR  18   // White cable
#define inaR   6
#define inbR   7


volatile long encoderRPos=0;
float newpositionR;
float oldpositionR = 0;
unsigned long newtime_encoder;
unsigned long oldtime_encoder = 0;
float wR_round;
float wR;

unsigned long tTime[4];


/********* Publishers *********/

//std_msgs::Float32 encoder;
//ros::Publisher encoder("encoder", &encoder_msg);