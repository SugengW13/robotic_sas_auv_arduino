/* Import Libraries */
#include <ros.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <robotic_sas_auv_ros/Sensor.h>
#include <robotic_sas_auv_ros/Actuator.h>

/* Declare Sensor */
Adafruit_BNO055 bno = Adafruit_BNO055();
MS5837 ms5837;

/* Declare Thruster Pin */
byte pin_thruster[8] = {3, 5, 7, 9, 12, 10, 8, 6};
Servo thruster[8];

/* Define ROS Node */
ros::NodeHandle node_arduino;

/* Define ROS Msgs */
robotic_sas_auv_ros::Sensor msg_sensor;

/* Define ROS Pubs */
ros::Publisher pub_sensor("sensor", &msg_sensor);

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
  Serial.begin(57600);

  Wire.begin();

  /* Sensor Check */
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  while (!ms5837.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(3000);
  }

  /* Init Sensor */
  ms5837.setModel(MS5837::MS5837_30BA);
  ms5837.setFluidDensity(1000);

  /* Redefine Thruster & Pin */
  for (int i = 0; i < 8; i++) {
    thruster[i].attach(pin_thruster[i]);
    thruster[i].writeMicroseconds(1500);
  }

  /* Init Node */
  node_arduino.initNode();

  /* Advertise Pubs */
  node_arduino.advertise(pub_sensor);

  /* Subscribe Subs */
  node_arduino.subscribe(sub_pwm_thruster);
}

void loop() {
  /* Read Sensor Value */
  sensors_event_t event;
  bno.getEvent(&event);
  ms5837.read();

  /* Store Sensor Value to Variable */
  float roll = event.orientation.x;
  float pitch = event.orientation.y;
  float yaw = event.orientation.z;

  float pressure = ms5837.pressure();
  float temperature = ms5837.temperature();
  float depth = ms5837.depth();
  float altitude = ms5837.altitude();

  /* Store Sensor Value to ROS Msgs */
  msg_sensor.roll = roll;
  msg_sensor.pitch = pitch;
  msg_sensor.yaw = yaw;
  msg_sensor.pressure = pressure;
  msg_sensor.temperature = temperature;
  msg_sensor.depth = depth;
  msg_sensor.altitude = altitude;

  /* Publish ROS Msgs */
  pub_sensor.publish(&msg_sensor);

  /* Spin ROS Node */
  node_arduino.spinOnce();

  delay(100);
}
