#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

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
  
  spin(double radia_angle) {
    angle = radiansToDegrees(radian_angle)
    double degree_change = (angle - calib)/mult_calib;
    servo.write(degree_change);
  }
};

Joint gripper;
Joint wrist;
Joint elbow;
Joint shoulder2;
Joint shoulder1;
Joint base;

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  char result[8]; // Buffer big enough for 7-character float
  for (byte i = 0; i < 2; ++i) {
    dtostrf(radiansToDegrees(cmd_msg.position[i]), 6, 2, result); // Leave room for too large numbers!
    nh.loginfo(result);
  }
  base.spin(cmd_msg.position[0])
  shoulder1.spin(cmd_msg.position[1])
  // TODO: calibrate elbow, wrist, grippper
  nh.loginfo("WIN");  
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  base = Joint(2, -90, 8/9);
  shoulder1 = Joint(4, 180, 8/9);
  // shoulder2 = Joint(6, 50, 8/9);
  // gripper = Joint(8, x, 8/9);
  // wrist = Joint(11, x, 8/9);
  // elbow = Joint(6, x, 8/9);

  delay(1);
}

void loop(){
  nh.spinOnce();
  for (int i = 0; i <= 160; i+=1) {
    base.spin(i);
    shoulder1.spin(i);
    delay(20);
  }
  delay(1000);
  for (int i = 160; i >= 0; i-=1) {
    base.spin(i);
    shoulder1.spin(i);
    delay(20);
  }
  delay(2000);
}