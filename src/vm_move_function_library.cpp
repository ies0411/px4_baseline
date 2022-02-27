#ifndef __MOVEBYVEL__
#define __MOVEBYVEL__

#include <vm_basic_command.h>
#include <vm_basic_cmd_library.h>

#if usedInDJI

/**
 * @brief Takeoff function
 * 
 */
bool Vm_Basic_Command::Take_Off(void){
   control_task.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_TAKEOFF;
   ROS_INFO_STREAM("Takeoff request sending ...");
   task_control_client.call(control_task);
   if(control_task.response.result == true){
      ROS_INFO_STREAM("Takeoff task successful");
   // Exception_Process();
 
      return true;
   }
   ROS_INFO_STREAM("takeoff task fail");
   return false;
}

/**
 * @brief position control function
 */

bool Vm_Basic_Command::Move_By_Pose(void){
   ROS_INFO_STREAM("Move by position offset request sending ...");
   // dji_osdk_ros::MoveOffset move_offset(y_goal,x_goal,z_goal,th_goal);
   control_task.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
   // control_task.request.pos_offset.clear();
   // control_task.request.pos_offset.push_back(move_offset.x);
   // control_task.request.pos_offset.push_back(move_offset.y);
   // control_task.request.pos_offset.push_back(move_offset.z);
   // control_task.request.yaw_params.clear();
   // control_task.request.yaw_params.push_back(move_offset.yaw);
   // control_task.request.yaw_params.push_back(move_offset.yaw_threshold);
   // control_task.request.yaw_params.push_back(move_offset.pos_threshold);
   // ROS_INFO_STREAM("call");
   // task_control_client.call(control_task);
   // ROS_INFO_STREAM("call complete");
   // return control_task.response.result;
  control_task.request.joystickCommand.y = x_goal;
  control_task.request.joystickCommand.x = y_goal;
  control_task.request.joystickCommand.z = z_goal;
  control_task.request.joystickCommand.yaw = th_goal;
  control_task.request.posThresholdInM   = 0.8;
  control_task.request.yawThresholdInDeg = 1.0;

  task_control_client.call(control_task);
  return control_task.response.result;
}


/**
 * @brief landing function
 */
bool Vm_Basic_Command::Landing(void){
   ROS_INFO_STREAM("Land request sending ...");
   control_task.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_LAND;
   task_control_client.call(control_task);
   if(control_task.response.result == true){
      ROS_INFO_STREAM("Land task successful");
      return true;
   }
   ROS_INFO_STREAM("Land task failed.");
   return false;
}
/**
 * @brief velocity control function
 */
bool Vm_Basic_Command::Move_By_Odom(const uint8_t &flight_type){
   ros::Rate Move_rate(10);
   linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
   #if TWO_POINT_DISTANCE
      z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   #endif
   info_pose[_INFO_X_GOAL]=x_goal;
   info_pose[_INFO_Y_GOAL]=y_goal;
   info_pose[_INFO_Z_GOAL]=z_goal;
   info_pose[_INFO_TH_GOAL]=th_goal;
   info_pose[_INFO_SAT_RANGE_W]=SAT_RANGE_YAW;
  
   ros::Time time_begin = ros::Time::now();
   while(ros::ok()){
      ros::Time time_end = ros::Time::now();
      ros::Duration duration = time_end - time_begin;
      if(duration.toSec()>TIMEOUT)return false;

      info_pose[_INFO_X]=x;
      info_pose[_INFO_Y]=y;
      info_pose[_INFO_Z]=z;
      info_pose[_INFO_TH_RAD]=th_rad;
      info_pose[_INFO_TH_DEG]=th_deg;
      info_pose[_INFO_SPEED]=speed;
      info_pose[_INFO_TURN]=turn;
      
     // &Linear_Flight_Algorithm(info_pose);
      double distance = Linear_Flight_Algorithm(info_pose);
      x_vel=info_pose[_INFO_X_VEL];
      y_vel=info_pose[_INFO_Y_VEL];
      z_vel=info_pose[_INFO_Z_VEL];
      th_vel=info_pose[_INFO_TH_VEL];

      speed = info_pose[_INFO_SPEED];
      turn = info_pose[_INFO_TURN];
  
      twist.linear.x = x_vel;
      twist.linear.y = y_vel;
      twist.linear.z = z_vel;
      twist.angular.z = th_vel;  

      #if ROTATION_POSE_CHECK
         if(fabs(th-th_goal)<0.5)break;
      #endif
      #if POSE_CHECK
          if(fabs(x-x_goal)<1.5 && fabs_y<1.5)break;
      #endif
      #if DIS_THRESHOLD_TYPE_CHECK
           if(distance < THREE_POINT_DISTANCE || Arrive_Flag==true){
               if(flight_type==_WAYPOINT_){
                  return true;
               }
               Arrive_Flag=true;
               twist.linear.x = ZERO_DOUBLE;
               twist.linear.y = ZERO_DOUBLE;
               twist.linear.z = ZERO_DOUBLE;
               pid_th_Kp=50;
               th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
               if(satisfy_th){
                  ROS_INFO_STREAM("Satisfy the Condition");
                  return true;
               }
         }
      #endif
      #if POSE_THRESHOLD_TYPE_CHECK
         if(fabs_x<SAT_RANGE_X && fabs_y<SAT_RANGE_Y && fabs_z<SAT_RANGE_Z ){
            twist.linear.x = ZERO_DOUBLE;
            twist.linear.y = ZERO_DOUBLE;
            twist.linear.z = ZERO_DOUBLE;
          
            if(satisfy_th){
               ROS_INFO_STREAM("Satisfy the Condition");
               return true;
            }
         }      
      #endif
     
       

      #if COMPLEDTE_ODOM_CHECK
         ROS_INFO("odom - x: %f y:%f z:%f th_deg:%f",x,y,z,th_deg);
      #endif
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      Move_rate.sleep();  
      
   }
   return false;
}
/**
 * @brief return to home process function
 */
bool Vm_Basic_Command::Return_To_Home_Process(const double &altitude){
   x_goal = rth_point.x;   //x-position
   y_goal = rth_point.y;   //y-position
   z_goal = altitude;   //z-position
   th_goal = 0.0;  
      
   if(Move_By_Odom(_SIMPLE_)==false){
      ROS_ERROR_STREAM("movebypose error");
      return false;
   }
   Landing();
}


 bool Vm_Basic_Command::GPSToOdom(const double &latitude, const double &longitude, const double &altitude ){
   double northing, easting;
   std::string zone;
   gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
   x_goal = easting;
   y_goal = northing;
   z_goal = altitude;
   th_goal = 0.0;
 }


bool Vm_Basic_Command::TrackingMarker(void){
   //if(Aruco_Flag){
      
   ros::Rate Move_rate(10);
   //linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);

   Marker_x_PID.PID_set(pid_dt,0.2,pid_min,0.01,pid_Kd,pid_Ki);
   Marker_y_PID.PID_set(pid_dt,0.2,pid_min,0.01,pid_Kd,pid_Ki);
   Marker_z_PID.PID_set(pid_dt,0.2,pid_min,0.01,pid_Kd,pid_Ki);


   ros::Time time_begin = ros::Time::now();
   while(ros::ok()){
      ROS_INFO_STREAM("marker_check");
      ros::Time time_end = ros::Time::now();
      ros::Duration duration = time_end - time_begin;
      if(duration.toSec()>TIMEOUT)return false;

     // &Linear_Flight_Algorithm(info_pose);
      double speed_x= Marker_x_PID.calculate(ZERO_DOUBLE,aruco_x);
      double speed_y= Marker_y_PID.calculate(1.0,aruco_y);
      double speed_z= Marker_z_PID.calculate(ZERO_DOUBLE,aruco_z);
      if(aruco_x>0)speed_x*=-1;
      if(aruco_y<3)speed_y*=-1;
      if(aruco_z>0)speed_z*=-1;
      ROS_INFO("%f %f %f",speed_x,speed_y,speed_z);
      twist.linear.x = speed_x;
      twist.linear.y = speed_y;
      twist.linear.z = speed_z;
      twist.angular.z = 0.0;  

      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      Move_rate.sleep();  
      
   }
  // }
   return false;
}



#endif




#if usedInGazebo

int Vm_Basic_Command::Landing(void){
   //
   linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
   z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
  if(Marker_Detect_flag){
     ros::Rate landing_rate(5);
     while(ros::ok()){
      ROS_INFO("landing");
      info_pose[_INFO_X_GOAL]=ZERO_DOUBLE;
      info_pose[_INFO_Y_GOAL]=ZERO_DOUBLE;
      info_pose[_INFO_Z_GOAL]=ZERO_DOUBLE;
      info_pose[_INFO_TH_GOAL]=ZERO_DOUBLE;
      info_pose[_INFO_X]=marker_x;
      info_pose[_INFO_Y]=marker_y;
      info_pose[_INFO_Z]=marker_z;
      info_pose[_INFO_TH_RAD]=marker_th;
      info_pose[_INFO_TH_DEG]=marker_th_deg;
      info_pose[_INFO_SAT_RANGE_W]=SAT_RANGE_YAW;
      info_pose[_INFO_SPEED]=speed;
      info_pose[_INFO_SPEED_Z]=Z_SPEED;
      info_pose[_INFO_TURN]=turn;
      info_pose[_INFO_SAT_RANGE_Z] = SAT_RANGE_Z;
      Linear_Flight_Algorithm(info_pose);


      twist.linear.x = info_pose[_INFO_X_VEL];
      twist.linear.y = info_pose[_INFO_Y_VEL];
      twist.linear.z = info_pose[_INFO_Z_VEL];
     
      twist.angular.x = ZERO_DOUBLE;
      twist.angular.y = ZERO_DOUBLE;
      twist.angular.z = info_pose[_INFO_TH_VEL];

 
      if(z<=SAT_LANDING_HEIGHT)break;
      //ROS_INFO("LANDING");
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      landing_rate.sleep();  
      }
   }
}

// void Vm_Basic_Command::Gps_sensor_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
 
//    Altitude = msg->altitude;
//    #if ALTITUDE_CHECK
//       ROS_INFO("Altitude : %f",Altitude);
//    #endif
// }

//filter gps odom & imu & encoding

int Vm_Basic_Command::Take_Off(void){
   z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   twist.linear.x = ZERO_DOUBLE;
   twist.linear.y = ZERO_DOUBLE;
   twist.linear.z = TAKEOFF_SPEED;

   twist.angular.x = ZERO_DOUBLE;
   twist.angular.y = ZERO_DOUBLE;
   twist.angular.z = ZERO_DOUBLE;
  // ROS_INFO("%f , %f",Altitude,Altitude_Dest);
   double takeoff_speed=ZERO_DOUBLE;
   double diff_pose_z;
   ros::Rate takeoff_rate(5);
   double z_pid;
   while(ros::ok()){
      
      if(z>=z_goal)break;
     
      diff_pose_z = z_goal-z;
      z_pid=z_speed_pid.calculate(ZERO_DOUBLE,diff_pose_z);
      if(takeoff_speed<z_pid)takeoff_speed+=0.05;
      else takeoff_speed=z_pid;

      // if(diff_pose_z>pose_info[_INFO_SAT_RANGE_Z])pose_info[_INFO_Z_VEL]=pose_info[_INFO_SPEED_Z];
      // else if(diff_pose_z<(-1*pose_info[_INFO_SAT_RANGE_Z]))pose_info[_INFO_Z_VEL]=(-1*pose_info[_INFO_SPEED_Z]);
      // else pose_info[_INFO_Z_VEL]=ZERO_DOUBLE;
      twist.linear.z=takeoff_speed;
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      takeoff_rate.sleep();  
       #if TAKEOFF_CHECK
         ROS_INFO("takeoff");
         ROS_INFO("v : %f,pid : %f,diff :%f",takeoff_speed,z_pid,diff_pose_z);
         //ROS_INFO("%f %f",Altitude,Altitude_Dest);
      #endif
   }
}
#endif








#endif



