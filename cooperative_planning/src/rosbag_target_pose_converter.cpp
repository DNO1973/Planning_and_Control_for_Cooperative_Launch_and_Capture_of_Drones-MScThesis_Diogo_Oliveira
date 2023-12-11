#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/Utils.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <smach_msgs/SmachContainerStatus.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseStamped.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>
#include <string>


#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <ros/package.h>




ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * shuttle = nullptr;
DroneLib::UAV * target = nullptr;

DroneLib::DroneInfo shuttle_drone_info;
DroneLib::DroneInfo target_drone_info;


//Controller Variables
geometry_msgs::Point desiredPosition;
geometry_msgs::Point desiredVelocityLinear;
double desiredHeading;

double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 1;
int proximity_radius_shuttle = 1;
int proximity_radius_target = 10;
int target_shuttle_vertical_closeness = 6;
int target_shuttle_horizontal_closeness = 4;
int verticalDistance = 2;

int targetReached = 0;
int reachedFlag = 0;

int informPointReached = 0;
int reachedFlagTarget = 0;

int targetReachedShuttle = 0;
int targetCloseFlag = 0;

int captureMoment = 0;
int captureStatus = 0;

double position_error[3][1];
double previous_error[3][1];

double Tprev;

int finished_trajectory = 0;


//hardcoded trajectory points
double shuttle_waiting_point[3][1];
double target_inform_point[3][1];
double shuttle_stop_area[3][1];

double referential_relative_to_shuttle_x;
double referential_relative_to_shuttle_y;
double referential_relative_to_shuttle_z;



int state_machine_status = 0;
float mpc_computation_time = 1111;
float yaw_target ;
float yaw_shuttle;





geometry_msgs::PoseStamped msg_to_send;

void callbackFunction(const geometry_msgs::PoseStamped::ConstPtr& msg){
      msg_to_send.header = msg->header;
      msg_to_send.pose.orientation =  msg->pose.orientation;
      msg_to_send.pose.position =  msg->pose.position;
      msg_to_send.pose.position.x = msg_to_send.pose.position.x + 5;
      msg_to_send.pose.position.y = msg_to_send.pose.position.y + 5;

      
      
      
}




int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_target_logger");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

  

    
  
    
    referential_relative_to_shuttle_x = 5;
    referential_relative_to_shuttle_y = 5;
    referential_relative_to_shuttle_z = 0;



    
    ros::Subscriber sub = nh->subscribe("/plane1/mavros/local_position/pose", 10, callbackFunction);
  ros::Publisher pub = nh->advertise<geometry_msgs::PoseStamped>("/correct_target_pose", 10);
    
    
    
    
    ros::Rate rate(100.0);
    ROS_WARN("Rosbag Target Pose Conversion Publisher");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

   



    while(ros::ok() ){
      
       

      pub.publish(msg_to_send);

     
      
   



        ros::spinOnce();
        rate.sleep();
    }




    return 0;


























}
