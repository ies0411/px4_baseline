#ifndef __VM_COMMAND__
#define __VM_COMMAND__

#include <vm_basic_command.h>
#include <vm_basic_cmd_library.h>


/**
 * @brief subcribe command line
 * @details 
 * cmd line : [takeoff , landing , landingbymarker , movebyvel , movebypose , waypoint , RTH]
 * @date '21.01.26
 * @author Ethan.lim  
 */

void Vm_Basic_Command::Keyboard_command_Callback(const std_msgs::String::ConstPtr &msg){
     std::istringstream iss(msg->data);
     std::string token;
     std::vector<std::string> param;
     Mission_flag = true;
      while(getline(iss,token,' ')){
         param.emplace_back(token);
      }
     
      if(param[0]=="takeoff"){
         if(!Take_Off())ROS_ERROR_STREAM("takeoff error");
         ros::Duration(2.0).sleep();
         if(!Exception_Process()){
            ros::Duration(5.0).sleep();
         }
      }
         
      else if(param[0]=="landing"){
         if(!Landing())ROS_ERROR_STREAM("landing error");
         ros::Duration(2.0).sleep();
      }
      
      else if(param[0]=="landingbymarker"){
         if(!Landingbymarker())ROS_ERROR_STREAM("landingbymarker error");
      }
      
      else if(param[0]=="movebyvel"){
         satisfy_th=false;
         Arrive_Flag=false;
         x_goal = stol(param[1]);   //x-position
         y_goal = stol(param[2]);   //y-position
         z_goal = stol(param[3]);   //z-position
         if(POSE_INPUT_EXCEPTION_MACRO(x_goal,y_goal,z_goal)){
            ROS_ERROR_STREAM("should input until 100 and z be upper than 0");
            return;
         }
         double tmp_th_goal = stol(param[4]);
         if(tmp_th_goal>360.0){
               ROS_ERROR_STREAM("input degree 0~360");
               return;
            }
         th_goal = DEG_TO_RAD(tmp_th_goal);
         #if GOTO_PARAM_CHECK
            ROS_INFO("%f, %f, %f, %f,",x_goal,y_goal,z_goal,tmp_th_goal);
         #endif
         if(Move_By_Odom(_SIMPLE_)==false)ROS_INFO_STREAM("movebyvel error");
         if(!Exception_Process()){
            ros::Duration(5.0).sleep();
         }
      }
      
      else if(param[0]=="movebypose"){
         x_goal = stol(param[1]);   //x-position
         y_goal = stol(param[2]);   //y-position
         z_goal = stol(param[3]);   //z-position
         if(POSE_INPUT_EXCEPTION_MACRO(x_goal,y_goal,z_goal)){
            ROS_ERROR_STREAM("should input until 100 and z be upper than 0");
            return;
         }
         double tmp_th_goal = stol(param[4]);
         if(tmp_th_goal>180.0){
            if(tmp_th_goal>360.0){
                  ROS_ERROR_STREAM("input degree 0~360");
                  return;
               }
            tmp_th_goal=tmp_th_goal-360.0;
         }
         th_goal = DEG_TO_RAD(tmp_th_goal);
         #if GOTO_PARAM_CHECK
            ROS_INFO("%f, %f, %f, %f,",x_goal,y_goal,z_goal,tmp_th_goal);
         #endif
         if(Move_By_Pose()==false)ROS_INFO_STREAM("movebypose error");
         if(!Exception_Process()){
            ros::Duration(5.0).sleep();
         }
      }
      
      else if(param[0]=="waypoint"){
       
         Waypoint temp;
         for(uint8_t index=1;index<param.size();index++){
         // for(auto &p : param){
            if(param[index]=="finish")break;
            
            if(index%4==1){
               temp.x_goal=stol(param[index]);
            }
            else if(index%4==2){
               temp.y_goal=stol(param[index]);
            }
            else if(index%4==3){
               temp.z_goal=stol(param[index]);
            }
            else if(index%4==0){
               if(POSE_INPUT_EXCEPTION_MACRO(temp.x_goal,temp.y_goal,temp.z_goal)){
                  ROS_ERROR_STREAM("should input until 100 and z be upper than 0");
                  return;
               }
               temp.yaw_goal=stol(param[index]);
               double tmp_th_goal = temp.yaw_goal;
               if(tmp_th_goal>360.0){
                  ROS_ERROR_STREAM("input degree 0~360");
               return;
               }
               temp.yaw_goal = DEG_TO_RAD(tmp_th_goal);
               waypoint_vector.emplace_back(temp);
               ROS_INFO("%lf %lf %lf %lf",temp.x_goal,temp.y_goal,temp.z_goal,temp.yaw_goal);
            }
            
         }
         for(uint8_t i=0 ;i<waypoint_vector.size();i++){
            x_goal = waypoint_vector[i].x_goal; y_goal = waypoint_vector[i].y_goal; z_goal = waypoint_vector[i].z_goal;
            satisfy_th=false;
            Arrive_Flag=false;
            ROS_INFO("%lf %lf %lf %lf",waypoint_vector[i].x_goal,waypoint_vector[i].y_goal,waypoint_vector[i].z_goal,waypoint_vector[i].yaw_goal);
            if(i==waypoint_vector.size()-1){
               Move_By_Odom(_FINAL_POSE_);
               return;
            } 
            Move_By_Odom(_WAYPOINT_);
         }
      
      }
      else if(param[0]=="RTH"){
         if(param.size()!=2){
            ROS_ERROR_STREAM("incorrect param number");
         }
         double altitude = stol(param[1]);
         if(altitude>50 || altitude <3){
            ROS_ERROR_STREAM("Input RTH Altitude from '3' to '50'");
            return;
         }
         satisfy_th=false;
         Arrive_Flag=false;
         if(Return_To_Home_Process(altitude))ROS_ERROR_STREAM("RTH_ERROR");
         
      }

      else if(param[0]=="GPS"){
         latitude = stol(param[1]);  
         longitude = stol(param[2]);   
         altitude = stol(param[3]); 
         satisfy_th=false;
         Arrive_Flag=false; 
         GPSToOdom(latitude,longitude,altitude);
      }

      else if(param[0]=="tracking"){
         TrackingMarker();
      }
      
      else{
         std::cout<<error_msg<<std::endl;
      }
      Mission_flag = false;

}

/**
 * @brief Init param & function
 * @details 
 * 1. obtain ctrl authority
 * 2. avoid enable
 * 3. Set Home Point
 * @date '21.01.26
 * @author Ethan.lim  
 */
bool Vm_Basic_Command::Init_Set(void){
  
   obtainCtrlAuthority.request.enable_obtain = true;
   obtain_ctrl_authority_client.call(obtainCtrlAuthority);


   // upward_avoid_req.request.enable = true;
   // enable_avoid_client.call(upward_avoid_req);

   // if(upward_avoid_req.response.result == false){
   //    ROS_ERROR_STREAM("Enable Avoid FAILED");
   // }
   // else{
   //    ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
   // }

   // upward_avoid_req.request.enable = true;
   // enable_upward_avoid_client.call(upward_avoid_req);

   // if(upward_avoid_req.response.result == false){
   //    ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
   // }

  
   // altitude_go_home.request.altitude = 5;
   // set_go_home_altitude_client.call(altitude_go_home);

   // if(altitude_go_home.response.result == false){
   //    ROS_ERROR_STREAM("Set altitude for go home FAILED");
   // }
   

   // set_current_point_as_home_client.call(home_set_req);

   // if(home_set_req.response.result == false){
   //    ROS_ERROR_STREAM("Set current position as Home, FAILED");
   // }

   rth_point.x = x; rth_point.y=y ; 

}

int main(int argc, char *argv[])
{
	//init ROS node
	ros::init(argc,argv,"Vm_Basic_Command");
   Vm_Basic_Command vm_basic_cmd;
   ros::Rate rate(10);
  
   while (ros::ok())
   { 
      #if usedInGazebo
      if(!vm_basic_cmd.Mission_flag){
         vm_basic_cmd.twist.linear.x = ZERO_DOUBLE;
         vm_basic_cmd.twist.linear.y = ZERO_DOUBLE;
         vm_basic_cmd.twist.linear.z = ZERO_DOUBLE;

         vm_basic_cmd.twist.angular.x = ZERO_DOUBLE;
         vm_basic_cmd.twist.angular.y = ZERO_DOUBLE;
         vm_basic_cmd.twist.angular.z = ZERO_DOUBLE;    
            vm_basic_cmd.cmd_vel_pub.publish(vm_basic_cmd.twist);
       }     
       #endif
         ros::spinOnce();
         rate.sleep();
 
   }

	
	return 0;
}


#endif
