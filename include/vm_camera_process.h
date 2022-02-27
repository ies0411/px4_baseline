#ifndef __CAMERA_H__
#define __CAMERA_H__


#include <dji_osdk_ros/Gimbal.h>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/SetupCameraH264.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <stdio.h>
#include <vector>
#include <math.h>


class Vm_Camera_Process
{
private:
   /* data */

   ros::NodeHandle nh;
   ros::Subscriber fpv_camera_stream_sub,main_camera_stream_sub,fpv_camera_h264_sub;
   
   ros::ServiceClient setup_camera_stream_client,setup_camera_h264_client;
  
   dji_osdk_ros::SetupCameraStream setupCameraStream_;
   dji_osdk_ros::SetupCameraH264 setupCameraH264_;

   ros::Publisher gimbal_pub;
   void mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
   void fpvCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
   void cameraH264CallBack(const sensor_msgs::Image::ConstPtr &msg);
   void Vm_Camera_Init(void);
protected:


public:
   Vm_Camera_Process(/* args */);
   ~Vm_Camera_Process();  
};

Vm_Camera_Process::Vm_Camera_Process(/* args */)
{    
    
  
   /**service**/
   setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
   setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");

   /**subscribe**/
   fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, &Vm_Camera_Process::cameraH264CallBack,this);
   fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, &Vm_Camera_Process::fpvCameraStreamCallBack,this);
   main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, &Vm_Camera_Process::mainCameraStreamCallBack,this);

   /**publish**/  
   gimbal_pub = nh.advertise<dji_osdk_ros::Gimbal>("dji_osdk_ros/gimbal_angle_cmd",10);

   Vm_Camera_Init();
  

}

Vm_Camera_Process::~Vm_Camera_Process()
{
   ROS_INFO("Basic_command Node close");
}






#endif