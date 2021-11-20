#include "encoder_example.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();
  //nh.advertise(encoder);

  pinMode(inaR, OUTPUT);
  pinMode(inbR, OUTPUT);
  
  pinMode(encaR, INPUT);           // MOTOR RIGHT
  digitalWrite(encaR, HIGH);       // turn on pullup resistor
  pinMode(encbR, INPUT);           // MOTOR RIGHT
  digitalWrite(encbR, HIGH);       // turn on pullup resistor

  attachInterrupt(digitalPinToInterrupt(encaR), doEncoderA, CHANGE);  // MOTOR ENCODER
  attachInterrupt(digitalPinToInterrupt(encbR), doEncoderB, CHANGE);  // EXTERNAL ENCODER
}

void loop() {

  unsigned long t = micros();

  if ((t - tTime[0]) >= (1000000 / ENCODER_PUBLISH_FREQUENCY))
  {
    newpositionR = encoderRPos;                      // MOTOR RIGHT
    newtime_encoder = micros();
    // 64*100 counts per revolution and reduction
    // La velocidad es de 100 rpm - 10.47 rad/s
    // Se da 4 pulsos por cuenta

    oldpositionR = newpositionR;
    oldtime_encoder = newtime_encoder;

    //encoder_msg.data = wR;
    //encoder.publish( &encoder_msg );
    tTime[0] = t;
  }

  nh.spinOnce();
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encaR) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == LOW) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == HIGH) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encbR) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encaR) == HIGH) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encaR) == LOW) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}
