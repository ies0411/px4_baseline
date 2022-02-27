/**
 * 
 * @author Ethan.lim
 * @brief this package name is vm basic flight
 *        realizing move by pose commanded by user
 *        and exception process 
 *        Next step : sensor fusion IMU & GPS using EKF , Lane traking and obstacle avoid navigation 
 * @version 0.1
 * @date 2021-01-26
 *  * 
 * 
 */


#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "vm_pid_control_library.h"
#include "../src/vm_pid_control_library.cpp"
// #include "../src/vm_basic_command.cpp"

#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
// #include <dji_osdk_ros/SetNewHomePoint.h>
// #include <dji_osdk_ros/AvoidEnable.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/Gimbal.h>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/SetupCameraH264.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <gps_common/conversions.h>

#include <stdio.h>
#include <unistd.h>
#include <termio.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <utility>

#include <map>

// be class

#define ZERO_INT 0
#define ZERO_DOUBLE 0.0
//#define TAKEOFF_SPEED  2
#define Z_SPEED 1

#define MARKER_CHECK 1
#define ROTATION_CHECK 0
#define ROTATION_POSE_CHECK 0
#define POSE_CHECK 0
#define POSE_THRESHOLD_TYPE_CHECK 0
#define DIS_THRESHOLD_TYPE_CHECK 1
#define MAP_CHECK 1
#define ALTITUDE_CHECK 0
#define TAKEOFF_CHECK 1
#define ODOM_CHECK 0
#define COMPLEDTE_ODOM_CHECK 1
#define LINEAR_TRANS_CHECK 1
#define GOTO_PARAM_CHECK 1
#define PID_CHECK 0
#define BATTERY_STATUS_CHECK 0


#define THREE_POINT_DISTANCE 1
#define TWO_POINT_DISTANCE 0

#define usedInGazebo 0
#define usedInDJI 1

#define  RAD_TO_DEG(RAD)   (RAD)*(180.f)/(M_PI)
#define  DEG_TO_RAD(DEG)   (DEG)*(M_PI)/(180.f)
#define  RAD_360        (2*M_PI)

#define SAT_RANGE_X      (1.5f)
#define SAT_RANGE_Y      (1.5f)
#define SAT_RANGE_Z      (1.5f)
#define SAT_RANGE_YAW    DEG_TO_RAD(5)
#define SAT_RANGE_DISTANCE (1.0f)

#define SAT_RANGE_LANDING  (1.5f)

#define SAT_LANDING_HEIGHT    (0.5f)

#define RTH 1
#define LANDING   2

#define VOLTAGE_THRESHOLD (45800.f)

#define INPUT_BOUNDARY  (100)
#define POSE_INPUT_EXCEPTION_MACRO(X,Y,Z) ((X > INPUT_BOUNDARY) || (Y > INPUT_BOUNDARY) || (Z > INPUT_BOUNDARY) ||(-1*X > INPUT_BOUNDARY) ||(-1*Y > INPUT_BOUNDARY) ||(-1*Z > INPUT_BOUNDARY)|| (Z < 0)) ? (1) : (0)
#define TIMEOUT         (100)

enum _Info_pose{
   _INFO_X,
   _INFO_Y,
   _INFO_Z,
   _INFO_X_GOAL,
   _INFO_Y_GOAL,
   _INFO_Z_GOAL,
   _INFO_TH_RAD,
   _INFO_TH_GOAL,
   _INFO_TH_DEG,
   _INFO_PITCH,
   _INFO_ROLL,
   _INFO_THRESHOLD_DISTANCE,
   _INFO_SAT_RANGE_W,
   _INFO_SAT_RANGE_X,
   _INFO_SAT_RANGE_Y,
   _INFO_SAT_RANGE_Z,
   _INFO_SPEED,
   _INFO_TURN,
   _INFO_X_VEL,
   _INFO_Y_VEL,
   _INFO_Z_VEL,
   _INFO_TH_VEL,
};

enum _Info_Linear_Flight_Type{
   _SIMPLE_,
   _WAYPOINT_,
   _FINAL_POSE_,
};


const std::string error_msg = R"(
   Command is not correct!!!!
)";

const std::string msg = R"(
  

   )";

class Vm_Basic_Command
{
private:
   /* data */
   struct Point{
      double x,y,z;
     
   };
   struct Waypoint{
      double x_goal,y_goal,z_goal,yaw_goal;
   };
   struct RTH_Point{
      double x,y,z=5.0,yaw=0.0;
  
   };
   struct Transfer_Position{
       double pre_pose_x,pre_pose_y,pre_pose_z;
       double transfer_pose_x,transfer_pose_y,transfer_pose_z;
   };
      
	ros::NodeHandle nh;
   ros::Subscriber gps_sub,keyboard_sub,gps_odom_sub,imu_odom_sub,ar_marker_sub
         ,battery_state_sub,gps_health_sub,flight_status_sub,aruco_marker_pose_sub;
         // fpv_camera_stream_sub,main_camera_stream_sub,fpv_camera_h264_sub;
   
   ros::ServiceClient task_control_client, set_go_home_altitude_client, get_go_home_altitude_client,enable_upward_avoid_client,
                        get_avoid_enable_client,emergency_brake_client,enable_avoid_client,
                      obtain_ctrl_authority_client;
                     //  setup_camera_stream_client,setup_camera_h264_client;
   dji_osdk_ros::FlightTaskControl control_task;
   dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
   
   // auto enable_upward_avoid_client   = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
   // dji_osdk_ros::SetAvoidEnable enable_upward_avoid_client;
   // dji_osdk_ros::GetAvoidEnable get_avoid_enable_client;
   // dji_osdk_ros::AvoidEnable avoid_req,upward_avoid_req;
   // dji_osdk_ros::GetGoHomeAltitude get_go_home_altitude_client;
   // dji_osdk_ros::SetGoHomeAltitude set_go_home_altitude_client;
   // dji_osdk_ros::EmergencyBrake emergency_brake_client;

   // dji_osdk_ros::SetupCameraStream setupCameraStream_;
   // dji_osdk_ros::SetupCameraH264 setupCameraH264_;

   dji_osdk_ros::SetAvoidEnable upward_avoid_req;
   dji_osdk_ros::GetAvoidEnable getAvoidEnable;

   ros::Publisher cmd_vel_pub;

   geometry_msgs::Twist twist;

   std::string key;
   std::string command_line(void);
   std::vector<Waypoint>  waypoint_vector;

   PID linear_speed_pid;
   PID th_w_pid;
   PID z_speed_pid;

   PID Marker_x_PID,Marker_y_PID,Marker_z_PID;
  // PID takeoff_pid;    //new change using pointer

   RTH_Point rth_point;

   bool bat_flag=false,gps_flag=false,flight_flag=false,Arrive_Flag=false;  bool satisfy_th = false;
   bool Aruco_Flag=false;
   uint8_t emergency_flag = RTH;

   bool Move_By_Pose(void);
   bool Take_Off(void);
   bool Landing(void);
   bool Move_By_Odom(const uint8_t &flight_type);
   void Gps_sensor_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
   void Keyboard_command_Callback(const std_msgs::String::ConstPtr &msg);
   void Gps_To_Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg);
   double Linear_Flight_Algorithm(std::map<int, double> &pose_info);
   void Imu_To_Odom_Callback(const sensor_msgs::Imu::ConstPtr &msg);
   void Ar_Marker_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
   void Transfer_RPY(std::map<int,double> &pose_info, Vm_Basic_Command::Transfer_Position &transfer_position);
   bool Landingbymarker(void);
   double Distance(const std::vector<std::pair<double,double>> &point);
   double Two_Point_Distance(const std::vector<std::pair<double,double>> &point);
   
   void Battery_State_Callback(const sensor_msgs::BatteryState::ConstPtr &msg);
   void GPS_State_Callback(const std_msgs::UInt8::ConstPtr &msg);
   void Flight_State_Callback(const std_msgs::UInt8::ConstPtr &msg);

   bool Exception_Process(void);
  
   bool Return_To_Home_Process(const double &altitude=10);

   bool GPSToOdom(const double &latitude, const double &longitude, const double &altitude );

   void ArucoMarkerPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
   
   bool TrackingMarker(void);
   // void mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);

protected:

   double x_vel=0,y_vel=0,z_vel =0,th_vel=0;
   double marker_x=0,marker_y=0,marker_z=0,marker_th=0,marker_th_deg=0;
   double aruco_x=0,aruco_y=0,aruco_z=0;

   double x = 0,y=0,z=0,th_rad=0,th_deg=0,roll_rad=0,pitch_rad=0;  
   double x_goal = 0,y_goal=0,z_goal=0, th_goal=0;
   double x_abs = 0, y_abs=0, z_abs=0, th_abs=0, roll_abs=0,pitch_abs=0; 

   double latitude,longitude,altitude;

   double speed = 0.0;
   double turn = 0.0;
   double marker_speed = 0.0;
   double marker_turn = 0.0;


   double pid_dt=0.1, pid_max=2.0, pid_min=0.0, pid_Kp=0.3, pid_Kd=0.5, pid_Ki=0.0 ;
   double pid_z_dt=0.1, pid_z_max=4.0, pid_z_min=0.0, pid_z_Kp=1.0, pid_z_Kd=0.1, pid_z_Ki=0.0 ;
   double pid_th_dt=0.1,pid_th_max=5.0,pid_th_min=0.0,pid_th_Kp=0.5,pid_th_Kd=0.1,pid_th_Ki=0.0;

   double z_vel_rate=0.05,speed_rate=0.01,turn_rate=0.01;
public:
   Vm_Basic_Command(/* args */);
   ~Vm_Basic_Command();
   
    bool Init_Set(void);
    bool Mission_flag = false,Odom_Start_flag= false ,Imu_Start_flag=false,Marker_Detect_flag=false;
      #if MAP_CHECK
     
      std::map<int, double> info_pose
      {
      {_INFO_X, x},
      {_INFO_Y, y},
      {_INFO_Z, z},
      {_INFO_X_GOAL, x_goal},
      {_INFO_Y_GOAL, y_goal},
      {_INFO_Z_GOAL, z_goal},
      {_INFO_TH_RAD, th_rad},
      {_INFO_TH_GOAL, th_goal},
      {_INFO_TH_DEG, th_deg},
      {_INFO_PITCH, ZERO_DOUBLE},
      {_INFO_ROLL, ZERO_DOUBLE},
      {_INFO_SAT_RANGE_W, SAT_RANGE_YAW},
      {_INFO_SAT_RANGE_X,_INFO_SAT_RANGE_X},
      {_INFO_SAT_RANGE_Y,_INFO_SAT_RANGE_Y},
      {_INFO_SAT_RANGE_Z,_INFO_SAT_RANGE_Z},
      {_INFO_THRESHOLD_DISTANCE,SAT_RANGE_DISTANCE},
      {_INFO_SPEED,speed},
      {_INFO_TURN,turn},
      {_INFO_X_VEL,x_vel},
      {_INFO_Y_VEL,y_vel},
      {_INFO_Z_VEL,z_vel},
      {_INFO_TH_VEL,th_vel},

      };
      #endif 
 

};

Vm_Basic_Command::Vm_Basic_Command(/* args */)
{
   //gps_sub = nh.subscribe("fix", 1, &Vm_Basic_Command::Gps_sensor_Callback,this);  

   /***publish**/
   cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("flight_ctrl_vel", 10); 
   
   /**subscribe**/
   aruco_marker_pose_sub =  nh.subscribe("vm_aruco_marker_pose",10,&Vm_Basic_Command::ArucoMarkerPose,this);

   battery_state_sub = nh.subscribe("dji_osdk_ros/battery_state",10, &Vm_Basic_Command::Battery_State_Callback,this);
   gps_health_sub = nh.subscribe("dji_osdk_ros/gps_health",10, &Vm_Basic_Command::GPS_State_Callback,this);
   flight_status_sub = nh.subscribe("dji_osdk_ros/flight_status",10,&Vm_Basic_Command::Flight_State_Callback,this);
   keyboard_sub = nh.subscribe("keyboard_command",1, &Vm_Basic_Command::Keyboard_command_Callback,this);
   gps_odom_sub = nh.subscribe("vm_odom",1,&Vm_Basic_Command::Gps_To_Odom_Callback,this);
   imu_odom_sub = nh.subscribe("dji_osdk_ros/imu",10,&Vm_Basic_Command::Imu_To_Odom_Callback,this);
   ar_marker_sub = nh.subscribe("ar_pose_marker",10,&Vm_Basic_Command::Ar_Marker_Callback,this);

   /**service**/
   task_control_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
   set_go_home_altitude_client = nh.serviceClient<dji_osdk_ros::SetGoHomeAltitude>("/set_go_home_altitude");
   // set_current_point_as_home_client = nh.serviceClient<dji_osdk_ros::SetNewHomePoint>("/set_current_point_as_home");
   // enable_avoid_client = nh.serviceClient<dji_osdk_ros::AvoidEnable>("/enable_avoid");
   get_avoid_enable_client = nh.serviceClient<dji_osdk_ros::GetAvoidEnable>("get_avoid_enable_status");

   enable_upward_avoid_client   = nh.serviceClient<dji_osdk_ros::SetAvoidEnable>("/set_upwards_avoid_enable");
   // enable_upward_avoid_client = nh.serviceClient<dji_osdk_ros::AvoidEnable>("/enable_upwards_avoid");
   obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");

   Init_Set();  
}

Vm_Basic_Command::~Vm_Basic_Command()
{
   ROS_INFO("Basic_command Node close");
}






#endif