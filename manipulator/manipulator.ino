/*
  Code for arduino to manage 6 DOF manipulator servos.
  Input signal passes by ROS /joint_states

  To run it you should connect servos to arduino pins specified in setup() method
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;

double radiansToDegrees(float position_radians) {
  position_radians = position_radians + 1.6;
  return position_radians * 57.2958;
}

struct Joint {
  Servo servo;
  int calib;
  double mult_calib;
  
  Joint() {}
  
  Joint(int pin, int calib, double mult_calib): calib(calib), mult_calib(mult_calib) {
    servo.attach(pin);
  }
  
  spin(double radian_angle) {
    double angle = radiansToDegrees(radian_angle);
    double degree_change = (angle + calib) * mult_calib;
    
    servo.write(degree_change);
  }
};

Joint gripper;
Joint wrist;
Joint elbow;
Joint shoulder2;
Joint shoulder1;
Joint base;

void servo_cb(const sensor_msgs::JointState& cmd_msg) {
  char result[8]; // Buffer big enough for 7-character float
  for (byte i = 0; i < 3; ++i) {
    dtostrf(radiansToDegrees(cmd_msg.position[i]), 6, 2, result); // Leave room for too large numbers!
    nh.loginfo(result);
  }
  base.spin(cmd_msg.position[0]);
  shoulder1.spin(cmd_msg.position[1]);
  shoulder2.spin(cmd_msg.position[2]);
  elbow.spin(cmd_msg.position[3]);

  nh.loginfo("finish step");
}

ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", servo_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  base = Joint(2, -70, 8.0/9);
  shoulder1 = Joint(4, -90, 8.0/9);
  shoulder2 = Joint(6, 10, 1);
  elbow = Joint(7, 0, 1); // TODO: calibrate elbow (elbow 25kg servo needs external supply)
  // wrist = Joint(11, x, 8/9);
  // gripper = Joint(8, x, 8/9);
  
  delay(1);
}

void loop() {
  nh.spinOnce();
}
