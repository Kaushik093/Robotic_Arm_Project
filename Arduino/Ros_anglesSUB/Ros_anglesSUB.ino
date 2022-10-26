#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Servo.h>

Servo servo1, servo2, servo3, servo4, servo5;
double pose[8];

ros::NodeHandle nh;


void servo_cb( const std_msgs::UInt8MultiArray& cmd_msg){
    for(short i = 0; i<=4; i++)
      pose[i]=cmd_msg.data[i];
      Serial.print("Servo angles recerived ");
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("/angles", &servo_cb);

void setup() {

  nh.initNode();
  nh.subscribe(sub);

  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
}

void loop() {
  nh.spinOnce();
  delay(1);
}