#include "Adafruit_VL53L1X.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//*************************************************************

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/int8_multi_array.h>


//*****************************************************************

void read_imu();
void read_tof();
void Servo_2_Angle(char servo,int angle);
void Reset();

/*Servo numbers*/
#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servo7 6
#define servo8 7
#define servo9 8


/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

Adafruit_VL53L1X ToF = Adafruit_VL53L1X();
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver(); 

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105


/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


//*************************************************************************************************

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher1 IMU
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rcl_timer_t timer;

// publisher2 ToF
rcl_publisher_t tof_publisher;
sensor_msgs__msg__Range tof_msg;

// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int8MultiArray servo_msg;

char *ssid = "asdf";
char *pwd = "0000asdf";
char *host_ip = "192.168.142.87";


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  Serial.println("in timer callback");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    read_imu();
    read_tof();
    rcl_publish(&imu_publisher, &imu_msg, NULL);
    rcl_publish(&tof_publisher, &tof_msg, NULL);
  }
  Serial.println("leaving timer callback");
}


void subscription_callback(const void *msgin)
{
    Serial.println("In subscription callback");

    // Cast the incoming message to the correct type
    const std_msgs__msg__Int8MultiArray *servo_msg = (const std_msgs__msg__Int8MultiArray *)msgin;

    for (char i = 0; i < 9; i++) {
        // Call the Servo_2_Angle function with the servo (char) and angle (int)
        // Servo_2_Angle(i, (int)(servo_msg->data.data[i]));  ??????????????????????????????????????????????????????????????

        // Serial.print("servo: ");
        // Serial.println((int)i);
        // Serial.print("angle: ");
        // Serial.println((int)(servo_msg->data.data[i]));
    }

    Serial.println("Leaving subscription callback");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start void setup");

  set_microros_wifi_transports(ssid, pwd, host_ip, 8888);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "micro_ros_STM32_node", "", &support);

  // create subscriber
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
      "servo/data");

  // create publisher
  rclc_publisher_init_default(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data");

  // create publisher
  rclc_publisher_init_default(
      &tof_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "tof/range");

  // create timer, called every 1000 ms to publish heartbeat
  const unsigned int timer_timeout = 1000;
  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback);

  // create executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  rclc_executor_add_subscription(&executor, &subscriber, &servo_msg, &subscription_callback, ON_NEW_DATA);


//********************************************************************************************************************

  Wire.begin(); // 21 SDA, 22 SCL (ESP32)  ||     PB3 SDA, PB10 SCL (STM32)
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties

  driver.begin();
  driver.setPWMFreq(50);
  
  mpu.initialize();

  if(mpu.testConnection() == false){
    // Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    // Serial.println("MPU6050 connection successful");
  }  

  /* Initializate and configure the DMP*/
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  //initialize IMU msg
  imu_msg.orientation.w = 0.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.linear_acceleration.x = 0.0;
  imu_msg.linear_acceleration.y = 0.0;
  imu_msg.linear_acceleration.z = 0.0;
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = 0.0;

  servo_msg.data.data = (int8_t *)malloc(9 * sizeof(int8_t));
  servo_msg.data.capacity = 9;
  servo_msg.data.size = 9;


  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // Serial.println("These are the Active offsets: ");
    // mpu.PrintActiveOffsets();
    // Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
  read_imu();

  // if (! ToF.begin()) {           ??????????????????????????????????????????????????????????????
  //   Serial.print(F("Error on init of VL sensor: "));
  //   Serial.println(ToF.vl_status);
  //   // while (1)       delay(10);
  // }

  // if (! ToF.startRanging()) {    ??????????????????????????????????????????????????????????????
  //   Serial.print(F("Couldn't start ranging: "));
  //   Serial.println(ToF.vl_status);
  //   // while (1)       delay(10);
  // }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  // ToF.setTimingBudget(200);      ??????????????????????????????????????????????????????????????

  Reset();
  delay(1500); 
  Serial.println("end void setup");
}

void loop()
{
  Serial.println("Start void loop");
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  Serial.println("end void loop");
}










//****************************************************************************************************

void read_imu(){
  Serial.println("Start read imu");
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) // Get the Latest packet 
  { 
    // Quaternions
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    imu_msg.orientation.w = q.w;
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;

    // Linear Acceleration
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    imu_msg.linear_acceleration.x = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.y = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.z = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

    // Angular Velocity
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    imu_msg.angular_velocity.x = ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;

    imu_msg.header.frame_id.data = "IMU_sensor";
  }
  Serial.println("end read imu");
}

void read_tof(){
  int16_t distance = 0;

  // if (ToF.dataReady()) {         ??????????????????????????????????????????????????????????????
  //   // new measurement for the taking!
  //   distance = ToF.distance();
  //   if (distance == -1) {
  //     // something went wrong!
  //     // Serial.print(F("Couldn't get distance: "));
  //     // Serial.println(ToF.vl_status);
  //     return;
  //   }
    tof_msg.range = distance;
    tof_msg.header.frame_id.data = "ToF_sensor"; 
    // Serial.print(F("Distance: "));
    // Serial.print(distance);
    // Serial.println(" mm");

    // data is read out, time for another reading!
    // ToF.clearInterrupt();        ??????????????????????????????????????????????????????????????
  }


  void Servo_2_Angle(char servo,int angle) {
  int val;
  switch(servo) {
    case servo1:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo2:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo3:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo4:
      val = map(angle, 0, 200, 400, 2775); // 375, 2780
      break;

    case servo5:
      val = map(angle, 0, 200, 400, 2785);
      break;

    case servo6:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo7:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo8:
      val = map(angle, 0, 200, 400, 2785);
      break;

    case servo9:
      val = map(angle, 0, 200, 400, 2780);
      break;
  }
  driver.writeMicroseconds(servo, val);
}


void Reset() {
   for (int i = 0; i < 9; i++) {
     Servo_2_Angle(i, 0);
   }
}
