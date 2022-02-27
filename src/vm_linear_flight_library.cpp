#ifndef __LINEAR_FLY_LIB__
#define __LINEAR_FLY_LIB__

#include <vm_basic_command.h>
#include <vm_basic_cmd_library.h>

/**
 * @brief linear flight algorithm function
 * @details go straight to goal position
 * getting a GPS & IMU data and transfer global pose to local pose 
 * using PD controller
 * @author Ethan.lim
 * @date '21.01.26 
 */


#if THREE_POINT_DISTANCE
double Vm_Basic_Command::Two_Point_Distance(const std::vector<std::pair<double,double>> &point){
   
  return sqrt(pow(point[0].first - point[0].second,2)+pow(point[1].first - point[1].second,2)); 
   
}

/**calculation between current position and goal position**/
double Vm_Basic_Command::Distance(const std::vector<std::pair<double,double>> &point){
   return sqrt(pow(point[0].first - point[0].second,2)+pow(point[1].first - point[1].second,2)+pow(point[2].first - point[2].second,2)); 
}

void Vm_Basic_Command::Transfer_RPY(std::map<int,double> &pose_info, Vm_Basic_Command::Transfer_Position &transfer_position){
   
   
   transfer_position.transfer_pose_x = transfer_position.pre_pose_x*cos(pose_info[_INFO_TH_RAD])*cos(pose_info[_INFO_PITCH]) 
                                      +transfer_position.pre_pose_y*(cos(pose_info[_INFO_TH_RAD])*sin(pose_info[_INFO_PITCH])*sin(pose_info[_INFO_ROLL])+sin(pose_info[_INFO_TH_RAD])*cos(pose_info[_INFO_ROLL]))
                                      +transfer_position.pre_pose_z*(sin(pose_info[_INFO_TH_RAD])*sin(pose_info[_INFO_ROLL])-cos(pose_info[_INFO_TH_RAD])*sin(pose_info[_INFO_PITCH])*cos(pose_info[_INFO_ROLL]));

   transfer_position.transfer_pose_y = transfer_position.pre_pose_x*(-1)*sin(pose_info[_INFO_TH_RAD])*cos(pose_info[_INFO_PITCH])
                                       +transfer_position.pre_pose_y*(cos(pose_info[_INFO_TH_RAD])*cos(pose_info[_INFO_ROLL])-sin(pose_info[_INFO_TH_RAD])*sin(pose_info[_INFO_PITCH])*sin(pose_info[_INFO_ROLL]))
                                       +transfer_position.pre_pose_z*(cos(pose_info[_INFO_ROLL])*sin(pose_info[_INFO_PITCH])*sin(pose_info[_INFO_TH_RAD])+cos(pose_info[_INFO_TH_RAD])*sin(pose_info[_INFO_ROLL]));
   
   transfer_position.transfer_pose_z = transfer_position.pre_pose_x*sin(pose_info[_INFO_PITCH])
                                       +transfer_position.pre_pose_y*(-1)*cos(pose_info[_INFO_PITCH])*sin(pose_info[_INFO_ROLL])
                                       +transfer_position.pre_pose_z*cos(pose_info[_INFO_PITCH])*cos(pose_info[_INFO_ROLL]);
}



double Vm_Basic_Command::Linear_Flight_Algorithm(std::map<int,double> &pose_info){
   double fabs_x = fabs(info_pose[_INFO_X]- info_pose[_INFO_X_GOAL]); 
   double fabs_y = fabs(info_pose[_INFO_Y]- info_pose[_INFO_Y_GOAL]);
   double fabs_z = fabs(info_pose[_INFO_Z]- info_pose[_INFO_Z_GOAL]);

   Vm_Basic_Command::Transfer_Position transfer_position;

   transfer_position.pre_pose_x= pose_info[_INFO_X];  transfer_position.pre_pose_y= pose_info[_INFO_Y];  transfer_position.pre_pose_z= pose_info[_INFO_Z];
   Transfer_RPY(pose_info,transfer_position);


   double transfer_pose_x = transfer_position.transfer_pose_x;
   double transfer_pose_y = transfer_position.transfer_pose_y;
   double transfer_pose_z = transfer_position.transfer_pose_z;


   transfer_position.pre_pose_x= pose_info[_INFO_X_GOAL];  transfer_position.pre_pose_y= pose_info[_INFO_Y_GOAL];  transfer_position.pre_pose_z= pose_info[_INFO_Z_GOAL];

   Transfer_RPY(pose_info,transfer_position);

   double transfer_pose_x_goal = transfer_position.transfer_pose_x;
   double transfer_pose_y_goal = transfer_position.transfer_pose_y;
   double transfer_pose_z_goal = transfer_position.transfer_pose_z;

   std::vector<std::pair<double,double>> point;

   point.emplace_back(std::make_pair(transfer_pose_x,transfer_pose_x_goal));
   point.emplace_back(std::make_pair(transfer_pose_y,transfer_pose_y_goal));
   point.emplace_back(std::make_pair(transfer_pose_z,transfer_pose_z_goal));

   double distance = Distance(point);
   double x_y_distance = Two_Point_Distance(point);


   double diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
   double diff_pose_y = transfer_pose_y_goal - transfer_pose_y;
   double diff_pose_z = transfer_pose_z_goal - transfer_pose_z;

   double tan_degree = atan2(diff_pose_y,diff_pose_x);
   double tan_degree_z = atan2(diff_pose_z,x_y_distance);
   
   double speed_pid= linear_speed_pid.calculate(ZERO_DOUBLE,distance);

   if(pose_info[_INFO_SPEED]<speed_pid)pose_info[_INFO_SPEED]+=speed_rate;
   else pose_info[_INFO_SPEED]=speed_pid;

   pose_info[_INFO_Z_VEL] = (pose_info[_INFO_SPEED]) * sin(tan_degree_z);
   pose_info[_INFO_Y_VEL] = (pose_info[_INFO_SPEED]) * cos(tan_degree_z)*sin(tan_degree);
   pose_info[_INFO_X_VEL] = (pose_info[_INFO_SPEED]) * cos(tan_degree_z)*cos(tan_degree);


   double fabs_th=fabs(pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD]);
   double diff_yaw=pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD];
   double turn_pid = th_w_pid.calculate(ZERO_DOUBLE,fabs(diff_yaw));
   if(pose_info[_INFO_TURN]<turn_pid)pose_info[_INFO_TURN]+=turn_rate;
   else pose_info[_INFO_TURN]=turn_pid;
   if(fabs_th>M_PI){
      if(RAD_360 - fabs_th < pose_info[_INFO_SAT_RANGE_W]){
         pose_info[_INFO_TH_VEL]=ZERO_DOUBLE; satisfy_th=true;
      }
      else{
         if(diff_yaw>ZERO_DOUBLE){
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }
         else{
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
         satisfy_th=false;  
      }
   }
   else{
      if(fabs_th<pose_info[_INFO_SAT_RANGE_W]){
          pose_info[_INFO_TH_VEL]=ZERO_DOUBLE; satisfy_th=true;
      }
      else{
         if(diff_yaw>ZERO_DOUBLE){
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
         else{
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }
         satisfy_th=false;         
      }
   }

   #if LINEAR_TRANS_CHECK
       
       //th_vel=0;
      ROS_INFO("diff_z : %f , diff_xy : %f ,tan : %f",diff_pose_z,x_y_distance,RAD_TO_DEG(tan_degree_z));
      ROS_INFO("trans x: %f , trans y : %f ,trans z : %f, x :%f , y :%f ,z:%f ,th:%f ",transfer_pose_x,transfer_pose_y,transfer_pose_z,x,y,z,th_deg);
      ROS_INFO("roll : %f , pitch : %f , yaw : %f",roll_rad,pitch_rad,th_rad);
      ROS_INFO("t_goal_x : %f , t_goal_y : %f , t_goal_z : %f ", transfer_pose_x_goal,transfer_pose_y_goal,transfer_pose_z_goal);
      ROS_INFO("vel  %f %f %f %f",pose_info[_INFO_X_VEL],pose_info[_INFO_Y_VEL],pose_info[_INFO_Z_VEL],pose_info[_INFO_TH_VEL]);
   #endif
   return distance;
   
}
#endif

#if TWO_POINT_DISTANCE

double Vm_Basic_Command::Distance(const std::vector<std::pair<double,double>> &point){
   
  return sqrt(pow(point[0].first - point[1].first,2)+pow(point[0].second - point[1].second,2)); 
   
}

bool Vm_Basic_Command::Linear_Flight_Algorithm(std::map<int,double> &pose_info){
   double fabs_x = fabs(info_pose[_INFO_X]- info_pose[_INFO_X_GOAL]); 
   double fabs_y = fabs(info_pose[_INFO_Y]- info_pose[_INFO_Y_GOAL]);
   double fabs_z = fabs(info_pose[_INFO_Z]- info_pose[_INFO_Z_GOAL]);

   double transfer_yaw = pose_info[_INFO_TH_RAD];  
   double transfer_pose_x = cos(transfer_yaw)*pose_info[_INFO_X] + sin(transfer_yaw)*pose_info[_INFO_Y];
   double transfer_pose_y = -sin(transfer_yaw)*pose_info[_INFO_X] + cos(transfer_yaw)*pose_info[_INFO_Y];
   
   double transfer_pose_x_goal = cos(transfer_yaw)*pose_info[_INFO_X_GOAL] + sin(transfer_yaw)*pose_info[_INFO_Y_GOAL] ;
   double transfer_pose_y_goal = -sin(transfer_yaw)*pose_info[_INFO_X_GOAL] + cos(transfer_yaw)*pose_info[_INFO_Y_GOAL] ; 
   double diff_pose_z = pose_info[_INFO_Z_GOAL]-pose_info[_INFO_Z];
  
   double diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
   double diff_pose_y = transfer_pose_y_goal - transfer_pose_y;

     
   std::vector<std::pair<double,double>> point;

   point.emplace_back(std::make_pair(transfer_pose_x,transfer_pose_y));
   point.emplace_back(std::make_pair(transfer_pose_x_goal,transfer_pose_y_goal));
   double distance = Distance(point);

   double tan_degree = atan2(diff_pose_y,diff_pose_x);
   double z_pid=z_speed_pid.calculate(ZERO_DOUBLE,diff_pose_z);


   if(diff_pose_z>pose_info[_INFO_SAT_RANGE_Z]){
      if(pose_info[_INFO_Z_VEL]<z_pid)pose_info[_INFO_Z_VEL]+=z_vel_rate;
      else pose_info[_INFO_Z_VEL]=z_pid;
   }
   else if(diff_pose_z<(-1*pose_info[_INFO_SAT_RANGE_Z])){
       if(pose_info[_INFO_Z_VEL]<z_pid)pose_info[_INFO_Z_VEL]-=z_vel_rate;
      else pose_info[_INFO_Z_VEL]=z_pid;
   }
   else pose_info[_INFO_Z_VEL]=ZERO_DOUBLE;

   #if PID_CHECK
      // PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
      PID pid = PID(0.1,1.0,0,0.1,0.01,0.5);
      double tmp = pid.calculate(0,20);
   #endif
  

   double speed_pid= linear_speed_pid.calculate(ZERO_DOUBLE,distance);
   if(pose_info[_INFO_SPEED]<speed_pid)pose_info[_INFO_SPEED]+=speed_rate;
   else pose_info[_INFO_SPEED]=speed_pid;
   pose_info[_INFO_Y_VEL] = (pose_info[_INFO_SPEED]) * sin(tan_degree);
   pose_info[_INFO_X_VEL] = (pose_info[_INFO_SPEED]) * cos(tan_degree);


   double fabs_th=fabs(pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD]);
   bool satisfy_th=false;
   double diff_yaw=pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD];
   double turn_pid = th_w_pid.calculate(ZERO_DOUBLE,fabs(diff_yaw));
   if(pose_info[_INFO_TURN]<turn_pid)pose_info[_INFO_TURN]+=turn_rate;
   else pose_info[_INFO_TURN]=turn_pid;
   if(fabs_th>M_PI){
      if(RAD_360 - fabs_th < pose_info[_INFO_SAT_RANGE_W]){
         pose_info[_INFO_TH_VEL]=ZERO_DOUBLE; satisfy_th=true;
      }
      else{
         if(diff_yaw>ZERO_DOUBLE){
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }
         else{
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
         satisfy_th=false;  
      }
   }
   else{
      if(fabs_th<pose_info[_INFO_SAT_RANGE_W]){
          pose_info[_INFO_TH_VEL]=ZERO_DOUBLE; satisfy_th=true;
      }
      else{
         if(diff_yaw>ZERO_DOUBLE){
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
         else{
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }
         satisfy_th=false;         
      }
   }

     
      #if ROTATION_POSE_CHECK
         if(fabs(th-th_goal)<0.5)break;
      #endif
      #if POSE_CHECK
          if(fabs(x-x_goal)<1.5 && fabs_y<1.5)break;
      #endif
      #if POSE_THRESHOLD_TYPE_CHECK
        
         if(fabs_x<SAT_RANGE_X && fabs_y<SAT_RANGE_Y && fabs_z<SAT_RANGE_Z ){
            twist.linear.x = ZERO_DOUBLE;
            twist.linear.y = ZERO_DOUBLE;
            twist.linear.z = ZERO_DOUBLE;
            if(satisfy_th){
               ROS_INFO("Meet The Condition");
               return true;
            }
         }
       
      #endif


   #if LINEAR_TRANS_CHECK
       
       //th_vel=0;
      ROS_INFO("trans x: %f , trans y : %f , x :%f , y :%f ,z:%f ,th:%f ",transfer_pose_x,transfer_pose_y,x,y,z,th_deg);
      ROS_INFO("speed : %f ,raq_speed : %f",speed,turn);
      //ROS_INFO("th_goal:%f , th:%f, th_abs:%f , diff_th:%f",RAD_TO_DEG(th_goal),th_deg,RAD_TO_DEG(th_abs),RAD_TO_DEG(transfer_yaw));
      ROS_INFO("vel  %f %f %f %f",pose_info[_INFO_X_VEL],pose_info[_INFO_Y_VEL],pose_info[_INFO_Z_VEL],pose_info[_INFO_TH_VEL]);
   #endif
   return false;
   
}
#endif

#endif
