#ifndef __CAMERA__
#define __CAMERA__

#include <vm_camera_process.h>

/**
 * @brief camera raw data & gimbal control process function
 * @details not yet completed
 * @author Ethan.lim
 * @date '21.01.26 
 */

void Vm_Camera_Process::Vm_Camera_Init(void){
    // setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
    // setupCameraStream_.request.start = 1;
    // setup_camera_stream_client.call(setupCameraStream_);

        setupCameraH264_.request.request_view = setupCameraH264_.request.FPV_CAMERA;
        setupCameraH264_.request.start        = 1;
        setup_camera_h264_client.call(setupCameraH264_);
        
        setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
        setupCameraStream_.request.start = 1;
        setup_camera_stream_client.call(setupCameraStream_);

//    setupCameraH264_.request.request_view = setupCameraH264_.request.MAIN_CAMERA;
//    setupCameraH264_.request.start        = 1;
//    setup_camera_h264_client.call(setupCameraH264_);
//    if(setupCameraH264_.response.result==false)ROS_ERROR_STREAM("cameraH264 FAILED");


    
}


void Vm_Camera_Process::mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg){
   
}

void Vm_Camera_Process::fpvCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg){
   
}

void Vm_Camera_Process::cameraH264CallBack(const sensor_msgs::Image::ConstPtr &msg){
   
}

int main(int argc, char *argv[])
{
	//init ROS node
    ros::init(argc,argv,"Vm_Camera_Process");
    Vm_Camera_Process vm_camera_process;
    ros::spin();
    return 0;
}


#endif
