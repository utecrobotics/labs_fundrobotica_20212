#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define MOTOR_FREQUENCY                 1000   //hz

#define inaR   6
#define inbR   7


unsigned long  oldtime_motor = 0;
unsigned long  newtime_motor;

float input = 0;
int motor_control = 0;

unsigned long tTime[4];


/********* Subscribers *********/

void motor_cb(const std_msgs::Float32& motor_msg);
ros::Subscriber<std_msgs::Float32> motor("motor", motor_cb);
