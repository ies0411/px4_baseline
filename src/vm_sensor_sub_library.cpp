#ifndef __SENSOR_SUB__
#define __SENSOR_SUB__

#include <vm_basic_command.h>
#include <vm_basic_cmd_library.h>

void Vm_Basic_Command::Imu_To_Odom_Callback(const sensor_msgs::Imu::ConstPtr &msg){
    tf::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
   );
   tf::Matrix3x3 m(q);
   double roll,pitch,yaw;
   m.getRPY(roll,pitch,yaw);
   if(yaw<0)yaw = RAD_360 + yaw;
  
   if(!Imu_Start_flag){
      th_abs = yaw;
      roll_abs=roll;
      pitch_abs=pitch;
   }
   else{
      th_rad = yaw-th_abs;
      roll_rad=roll-roll_abs;
      pitch_rad=pitch-pitch_abs;
      if(th_rad<0)th_rad=RAD_360+th_rad;
      th_deg=RAD_TO_DEG(th_rad);
   }  
   #if ODOM_CHECK
         ROS_INFO("roll : %f , pitch : %f , yaw : %f",RAD_TO_DEG(roll),RAD_TO_DEG(pitch),RAD_TO_DEG(yaw));
         //ROS_INFO("x_abs : %f , y_abs : %f, z_abs : %f",x_abs,y_abs,z_abs);
   #endif
   Imu_Start_flag=true;
}

void Vm_Basic_Command::Gps_To_Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg){   
  
   if(!Odom_Start_flag){
      x_abs = msg->pose.pose.position.x;
      y_abs = msg->pose.pose.position.y;
      z_abs = msg->pose.pose.position.z;
   }
   else{
      x = msg->pose.pose.position.x - x_abs ;
      y = msg->pose.pose.position.y - y_abs;
      z = msg->pose.pose.position.z -z_abs;

      #if ODOM_CHECK
         ROS_INFO("ODOM_CHECK , x : %f , y : %f , z : %f",x,y,z);
       //  ROS_INFO("x_abs : %f , y_abs : %f, z_abs : %f",x_abs,y_abs,z_abs);
      #endif
   }

   Odom_Start_flag=1;
}




#endif