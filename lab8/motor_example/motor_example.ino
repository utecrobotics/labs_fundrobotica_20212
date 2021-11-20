#include "motor_example.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();

  nh.subscribe(motor);
  pinMode(inaR, OUTPUT);
  pinMode(inbR, OUTPUT);
  
  digitalWrite(inaR, LOW);       // MOTOR
  digitalWrite(inbR, LOW);
}

void loop() {

  unsigned long t = micros();

  if ((t - tTime[0]) >= (1000000 / MOTOR_FREQUENCY))
  {
    newtime_motor = micros();

    motor_control = 0;
    //motor_control = (int) input;
  
    if (motor_control == 0) {
      digitalWrite(inaR, LOW);
      digitalWrite(inbR, LOW);
    }
    else if (motor_control > 0 and motor_control <= 255) {
      analogWrite(inaR, motor_control);
      digitalWrite(inbR, LOW);
    }
    else if (motor_control > 255) {
      digitalWrite(inaR, HIGH);
      digitalWrite(inbR, LOW);
    }
    else if (motor_control < 0 and motor_control >= -255) {
      digitalWrite(inaR, LOW);
      analogWrite(inbR, motor_control*(-1));
    }
    else if (motor_control < -255) {
      digitalWrite(inaR, LOW);
      digitalWrite(inbR, HIGH);
    }
    
    oldtime_motor = newtime_motor;
    tTime[0] = t;
  }

  nh.spinOnce();
}


void motor_cb(const std_msgs::Float32& motor_msg) {

  input = motor_msg.data;
}
