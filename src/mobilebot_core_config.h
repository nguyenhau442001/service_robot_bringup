#include "ros/ros.h"
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

// #include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>


#include <sstream>

#include <cstdlib>
#include <math.h>
#include <chrono>

#include "mobilebot_core.h"        // Declare kinetic/dynamic of robot 
#define FIRMWARE_VER "1.2.6"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define constrain(value, min, max) (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);
void imuCallback(const sensor_msgs::Imu& imu_msg);

// Function prototypes
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

// // Messages
turtlebot3_msgs::SensorState sensor_state_msg;
turtlebot3_msgs::VersionInfo version_info_msg;
nav_msgs::Odometry odom;
sensor_msgs::JointState joint_states;

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
geometry_msgs::TransformStamped odom_tf;
// tf::TransformBroadcaster tf_broadcaster; // Due to this one make can not connect

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
// Turtlebot3MotorDriver motor_driver;

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
// Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
// Turtlebot3Controller controllers;
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};

float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};


/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
// Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;
