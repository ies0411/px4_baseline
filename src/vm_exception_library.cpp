#ifndef __EXCEPTION_PRO__
#define __EXCEPTION_PRO__

#include <vm_basic_command.h>
#include <vm_basic_cmd_library.h>

/**
 * @brief Exception Process Function
 * @details Checking battery , flight  and gps health status 
 * sending signal if batter lower than 10% ,  bad flight status , bad GPS signal(under the 3)
 * then RTH or Emergency landing according to param
 * @author Ethan.lim
 * @date '21.01.26 
 */

#if usedInDJI

void Vm_Basic_Command::Battery_State_Callback(const sensor_msgs::BatteryState::ConstPtr &msg){
    #if BATTERY_STATUS_CHECK
        ROS_INFO("power_supply_health : %d \n vol : %f \n status : %d",msg->power_supply_health,msg->voltage,msg->power_supply_status);
    #endif
   //((msg->power_supply_health!=1&&msg->power_supply_health!=0)||(msg->voltage<VOLTAGE_THRESHOLD)||msg->percentage < 10 ) ? bat_flag=true : bat_flag=false;
}
void Vm_Basic_Command::GPS_State_Callback(const std_msgs::UInt8::ConstPtr &msg){
    (msg->data < 3) ? gps_flag=true : gps_flag=false ;
}
void Vm_Basic_Command::Flight_State_Callback(const std_msgs::UInt8::ConstPtr &msg){

}

bool Vm_Basic_Command::Exception_Process(void){
     if(gps_flag==true || bat_flag==true){
         ROS_INFO("emergency_landing(gps : %d , bat : %d)",gps_flag,bat_flag);
         if(emergency_flag==RTH){
            ROS_INFO("RTH_Request");
            if(Return_To_Home_Process()){
                ROS_ERROR_STREAM("RTH_ERROR");
                Landing();
            }
         }
         else if(emergency_flag==LANDING){
             Landing();
         }
        
        return false;
        
    }
   
        return true;
}
#endif

#endif