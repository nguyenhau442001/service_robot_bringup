#include "mobilebot_core_config.h" //Declare ros node


unsigned long millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
} // Oke

ros::Time now()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    return ros::Time(seconds.count(), nanoseconds.count());
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

void loginfo(const std::string& message)
{
   ROS_INFO("%s",message.c_str());
}

void imuCallback(const sensor_msgs::Imu& imu_msg)
{
  // maybe not use
}
/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
// //   char log_msg[50];

// //   (void)(reset_msg);

// //   sprintf(log_msg, "Start Calibration of Gyro");
// //   loginfo(log_msg);

// // //   sensors.calibrationGyro();

// //   sprintf(log_msg, "Calibration End");
// //   loginfo(log_msg);

// //   initOdom();

// //   sprintf(log_msg, "Reset Odometry");
// //   loginfo(log_msg);
}

// /*******************************************************************************
// * Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
// *******************************************************************************/
// void publishSensorStateMsg(void)
// {
//   bool dxl_comm_result = false;

//   sensor_state_msg.header.stamp = rosNow();

// //   dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

//   if (dxl_comm_result == true)
//     updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
//   else
//     return;

// //   sensor_state_pub.publish(&sensor_state_msg);
// }

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = std::vector<double>(joint_states_pos, joint_states_pos + WHEEL_NUM);
  joint_states.velocity = std::vector<double>(joint_states_vel, joint_states_vel + WHEEL_NUM);

}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
// void publishDriveInformation(void)
// {
//   unsigned long time_now = millis();
//   unsigned long step_time = time_now - prev_update_time;

//   prev_update_time = time_now;
//   ros::Time stamp_now = rosNow();

//   // calculate odometry
//   calcOdometry((double)(step_time * 0.001));

//   // odometry
//   updateOdometry();
//   odom.header.stamp = stamp_now;
// //   odom_pub.publish(&odom);

//   // odometry tf
//   updateTF(odom_tf);
//   odom_tf.header.stamp = stamp_now;
//   tf_broadcaster.sendTransform(odom_tf);

//   // joint states
// //   updateJointStates();
//   joint_states.header.stamp = stamp_now;
// //   joint_states_pub.publish(&joint_states);
// }

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
// void updateTFPrefix(bool isConnected)
// {
//   static bool isChecked = false;
//   char log_msg[50];

//   if (isConnected)
//   {
//     if (isChecked == false)
//     {
//     //   nh.getParam("~tf_prefix", &get_tf_prefix);

//       if (!strcmp(get_tf_prefix, ""))
//       {
//         sprintf(odom_header_frame_id, "odom");
//         sprintf(odom_child_frame_id, "base_footprint");  

//         sprintf(imu_frame_id, "imu_link");
//         sprintf(mag_frame_id, "mag_link");
//         sprintf(joint_state_header_frame_id, "base_link");
//       }
//       else
//       {
//         strcpy(odom_header_frame_id, get_tf_prefix);
//         strcpy(odom_child_frame_id, get_tf_prefix);

//         strcpy(imu_frame_id, get_tf_prefix);
//         strcpy(mag_frame_id, get_tf_prefix);
//         strcpy(joint_state_header_frame_id, get_tf_prefix);

//         strcat(odom_header_frame_id, "/odom");
//         strcat(odom_child_frame_id, "/base_footprint");

//         strcat(imu_frame_id, "/imu_link");
//         strcat(mag_frame_id, "/mag_link");
//         strcat(joint_state_header_frame_id, "/base_link");
//       }

//     //   sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
//     //   loginfo(log_msg);

//     //   sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
//     //   loginfo(log_msg);

//       isChecked = true;
//     }
//   }
//   else
//   {
//     isChecked = false;
//   }
// }

/*******************************************************************************
* Update the odometry
*******************************************************************************/
// void updateOdometry(void)
// {
//   odom.header.frame_id = odom_header_frame_id;
//   odom.child_frame_id  = odom_child_frame_id;

//   odom.pose.pose.position.x = odom_pose[0];
//   odom.pose.pose.position.y = odom_pose[1];
//   odom.pose.pose.position.z = 0;

//   tf::Quaternion odom_quat = tf::createQuaternionFromYaw(odom_pose[2]);
//   geometry_msgs::Quaternion orientation;
//   tf::quaternionTFToMsg(odom_quat, orientation);
//   odom.pose.pose.orientation = orientation;

//   odom.twist.twist.linear.x  = odom_vel[0];
//   odom.twist.twist.angular.z = odom_vel[2];
// }



/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
// void updateTF(geometry_msgs::TransformStamped& odom_tf)
// {
//   odom_tf.header = odom.header;
//   odom_tf.child_frame_id = odom.child_frame_id;
//   odom_tf.transform.translation.x = odom.pose.pose.position.x;
//   odom_tf.transform.translation.y = odom.pose.pose.position.y;
//   odom_tf.transform.translation.z = odom.pose.pose.position.z;
//   odom_tf.transform.rotation      = odom.pose.pose.orientation;
// }

/*******************************************************************************
* Update motor information
*******************************************************************************/
// void updateMotorInfo(int32_t left_tick, int32_t right_tick)
// {
//   int32_t current_tick = 0;
//   static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
//   if (init_encoder)
//   {
//     for (int index = 0; index < WHEEL_NUM; index++)
//     {
//       last_diff_tick[index] = 0;
//       last_tick[index]      = 0;
//       last_rad[index]       = 0.0;

//       last_velocity[index]  = 0.0;
//     }  

//     last_tick[LEFT] = left_tick;
//     last_tick[RIGHT] = right_tick;

//     init_encoder = false;
//     return;
//   }

//   current_tick = left_tick;

//   last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
//   last_tick[LEFT]      = current_tick;
//   last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

//   current_tick = right_tick;

//   last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
//   last_tick[RIGHT]      = current_tick;
//   last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
// }

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
// bool calcOdometry(double diff_time)
// {
//   float* orientation;
//   double wheel_l, wheel_r;      // rotation value of wheel [rad]
//   double delta_s, theta, delta_theta;
//   static double last_theta = 0.0;
//   double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
//   double step_time;

//   wheel_l = wheel_r = 0.0;
//   delta_s = delta_theta = theta = 0.0;
//   v = w = 0.0;
//   step_time = 0.0;

//   step_time = diff_time;

//   if (step_time == 0)
//     return false;

//   wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
//   wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

//   if (isnan(wheel_l))
//     wheel_l = 0.0;

//   if (isnan(wheel_r))
//     wheel_r = 0.0;

//   delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
//   theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  

//   // orientation = sensors.getOrientation();
//   // theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
//   //               0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

//   delta_theta = theta - last_theta;

//   // compute odometric pose
//   odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
//   odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
//   odom_pose[2] += delta_theta;

//   // compute odometric instantaneouse velocity

//   v = delta_s / step_time;
//   w = delta_theta / step_time;

//   odom_vel[0] = v;
//   odom_vel[1] = 0.0;
//   odom_vel[2] = w;

//   last_velocity[LEFT]  = wheel_l / step_time;
//   last_velocity[RIGHT] = wheel_r / step_time;
//   last_theta = theta;

//   return true;
// }

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
// void updateVariable(bool isConnected)
// {
//   static bool variable_flag = false;
  
//   if (isConnected)
//   {
//     if (variable_flag == false)
//     {      
//       sensors.initIMU();
//       initOdom();

//       variable_flag = true;
//     }
//   }
//   else
//   {
//     variable_flag = false;
//   }
// }
/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  std::string name             = NAME;
  std::string firmware_version = FIRMWARE_VER;
  std::string bringup_log      = "This core(v" + firmware_version + ") is compatible with " + name;


    if (log_flag == false)
    {      
        sprintf(log_msg, "--------------------------");
        loginfo(log_msg);

        sprintf(log_msg, "Connected to Mini PC and brought up successfully, no longer using AGV controller board." );
        loginfo(log_msg);

        sprintf(log_msg, "%s", bringup_log.c_str());
        loginfo(log_msg);

        sprintf(log_msg, "--------------------------");
        loginfo(log_msg);

        log_flag = true;
    }

}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
// void initOdom(void)
// {
//   init_encoder = true;

//   for (int index = 0; index < 3; index++)
//   {
//     odom_pose[index] = 0.0;
//     odom_vel[index]  = 0.0;
//   }

//   odom.pose.pose.position.x = 0.0;
//   odom.pose.pose.position.y = 0.0;
//   odom.pose.pose.position.z = 0.0;

//   odom.pose.pose.orientation.x = 0.0;
//   odom.pose.pose.orientation.y = 0.0;
//   odom.pose.pose.orientation.z = 0.0;
//   odom.pose.pose.orientation.w = 0.0;

//   odom.twist.twist.linear.x  = 0.0;
//   odom.twist.twist.angular.z = 0.0;
// }

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
// void initJointStates(void)
// {
//   static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

//   joint_states.header.frame_id = joint_state_header_frame_id;
//   joint_states.name            = joint_states_name;

//   joint_states.name_length     = WHEEL_NUM;
//   joint_states.position_length = WHEEL_NUM;
//   joint_states.velocity_length = WHEEL_NUM;
//   joint_states.effort_length   = WHEEL_NUM;
// }

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "mini_pc_control");

      /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle nh;

    /*******************************************************************************
    * Subscriber
    *******************************************************************************/
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel",1000,commandVelocityCallback); // OK
    ros::Subscriber reset_sub = nh.subscribe("reset", 1000, resetCallback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 1000, imuCallback); 

    /*******************************************************************************
    * Publisher
    *******************************************************************************/
    // Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
    ros::Publisher sensor_state_pub = nh.advertise<turtlebot3_msgs::SensorState>("sensor_state", 1000);

    // Version information of Turtlebot3
    ros::Publisher version_info_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("firmware_version", 1000); //OK

    // Odometry of Turtlebot3
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

    // // Joint(Dynamixel) state of Turtlebot3
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    // ros::Rate loop_rate(10); // The parameter in looprate is frequency (Hz)
    int count = 0;  


    while (ros::ok())
    {
        uint32_t t = millis();
        // loop_rate.sleep();

        if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
        {
          // publishDriveInformation();
          tTime[2] = t;
        }

        if ((t-tTime[4]) >= (1000 /VERSION_INFORMATION_PUBLISH_FREQUENCY))
        {
          // Publish version infor
          publishVersionInfoMsg();
          version_info_pub.publish(version_info_msg);
          tTime[4] = t;
        }

        sendLogMsg();
        
        ros::spinOnce();

    }
    
}
