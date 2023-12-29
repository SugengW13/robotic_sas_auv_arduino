#include <Servo.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <robotic_sas_auv_ros/Actuator.h>

/* Declare Thruster Pin */
byte pin_thruster[8] = {3, 5, 7, 9, 12, 10, 8, 6};Servo thruster[8];

/* Define ROS Node */
ros::NodeHandle node_arduino;

/* Define ROS Msgs */
std_msgs::Bool msg_is_start;

/* Define ROS Pubs */
ros::Publisher pub_is_start("is_start", &msg_is_start);

/* ROS Subs Callback */
void cb_pwm_thruster(const robotic_sas_auv_ros::Actuator& pwm) {
  thruster[0].writeMicroseconds(pwm.thruster_1);
  thruster[1].writeMicroseconds(pwm.thruster_2);
  thruster[2].writeMicroseconds(pwm.thruster_3);
  thruster[3].writeMicroseconds(pwm.thruster_4);
  thruster[4].writeMicroseconds(pwm.thruster_5);
  thruster[5].writeMicroseconds(pwm.thruster_6);
  thruster[6].writeMicroseconds(pwm.thruster_7);
  thruster[7].writeMicroseconds(pwm.thruster_8);
}

/* Define Ros Subs */
ros::Subscriber<robotic_sas_auv_ros::Actuator> sub_pwm_thruster("/nuc/pwm_actuator", &cb_pwm_thruster);

void setup() {
  /* Redefine Thruster & Pin */
  for (int i = 0; i < 8; i++) {
    thruster[i].attach(pin_thruster[i]);
    thruster[i].writeMicroseconds(1500);
  }

  /* Init Node */
  node_arduino.initNode();
  
  /* Subscribe Subs */
  node_arduino.advertise(pub_is_start);  

  /* Subscribe Subs */
  node_arduino.subscribe(sub_pwm_thruster);

  msg_is_start.data = false;
  pub_is_start.publish(&msg_is_start);
}

void loop() {
  msg_is_start.data = true;
  pub_is_start.publish(&msg_is_start);

  node_arduino.spinOnce();

  delay(100);
}
