#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>


//#include <std_msgs/msg/string.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#include <M5StickCPlus2.h>

rcl_publisher_t battery_publisher;
rcl_publisher_t buttonA_publisher;
rcl_publisher_t buttonB_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t tf_publisher;
std_msgs__msg__Int32 battery_msg;
std_msgs__msg__Int32 buttonA_msg;
std_msgs__msg__Int32 buttonB_msg;
sensor_msgs__msg__Imu imu_msg;
//geometry_msgs__msg__TransformStamped imu_transform;
geometry_msgs__msg__TransformStamped fixed_transform;
tf2_msgs__msg__TFMessage tf_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

void setup() {
  set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "HOST IP", PORT); 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher 1
  RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "battery/voltage"));

    // create publisher 2
  RCCHECK(rclc_publisher_init_default(
    &buttonA_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "button/A_status"));

    // create publisher 3
  RCCHECK(rclc_publisher_init_default(
    &buttonB_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "button/B_status"));

    // create publisher 4
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));

    // Initialize the TF publisher
  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // Initialize IMU message
  imu_msg.orientation_covariance[0] = -1; // Mark orientation as unavailable
  imu_msg.header.frame_id.data = (char*)"imu_frame";
  imu_msg.header.frame_id.size = strlen("imu_frame");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // Setup fixed transform (map -> base_link)
  fixed_transform.header.frame_id.data = (char *)"map"; // Fixed frame
  fixed_transform.header.frame_id.size = strlen("map");
  fixed_transform.header.frame_id.capacity = fixed_transform.header.frame_id.size + 1;

  fixed_transform.child_frame_id.data = (char *)"base_link"; // Root frame
  fixed_transform.child_frame_id.size = strlen("base_link");
  fixed_transform.child_frame_id.capacity = fixed_transform.child_frame_id.size + 1;

  // Define static transform values (adjust as needed)
  fixed_transform.transform.translation.x = 0.0;
  fixed_transform.transform.translation.y = 0.0;
  fixed_transform.transform.translation.z = 0.0;
  fixed_transform.transform.rotation.x = 0.0;
  fixed_transform.transform.rotation.y = 0.0;
  fixed_transform.transform.rotation.z = 0.0;
  fixed_transform.transform.rotation.w = 1.0;

  // Add the fixed transform to the tf message
  tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(1 * sizeof(geometry_msgs__msg__TransformStamped));
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;

  tf_msg.transforms.data[0] = fixed_transform;

  auto cfg = M5.config();
  StickCP2.begin(cfg);
  StickCP2.Display.setRotation(1);
  StickCP2.Display.setTextColor(GREEN);
  StickCP2.Display.setTextDatum(middle_center);
  StickCP2.Display.setTextFont(&fonts::Orbitron_Light_24);
  StickCP2.Display.setTextSize(1);
}

void loop() {
    StickCP2.update();
    StickCP2.Display.clear();
    auto imu_update = StickCP2.Imu.update();
    int vol = StickCP2.Power.getBatteryVoltage();
    battery_msg.data = vol;
    StickCP2.Display.setCursor(10, 30);
    StickCP2.Display.printf("BAT: %dmV \n", vol);

    int stateA = StickCP2.BtnA.getState();
    buttonA_msg.data = stateA;
    StickCP2.Display.printf("btnA state: %d \n", stateA);
    int stateB = StickCP2.BtnB.getState();
    buttonB_msg.data = stateB;
    StickCP2.Display.printf("btnB state: %d \n", stateB);
    
    auto data = StickCP2.Imu.getImuData();
    imu_msg.linear_acceleration.x = data.accel.x;
    imu_msg.linear_acceleration.y = data.accel.y;
    imu_msg.linear_acceleration.z = data.accel.z;
    imu_msg.angular_velocity.x = data.gyro.x;
    imu_msg.angular_velocity.y = data.gyro.y;
    imu_msg.angular_velocity.z = data.gyro.z;
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    // Publish the fixed frame transform
    fixed_transform.header.stamp.sec = millis() / 1000;
    fixed_transform.header.stamp.nanosec = (millis() % 1000) * 1000000;

    tf_msg.transforms.data[0] = fixed_transform;
    
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    RCSOFTCHECK(rcl_publish(&buttonA_publisher, &buttonA_msg, NULL));
    RCSOFTCHECK(rcl_publish(&buttonB_publisher, &buttonB_msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
    //delay(100);
}
