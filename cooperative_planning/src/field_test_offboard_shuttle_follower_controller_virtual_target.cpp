#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/Utils.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>



#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>
#include <ros/package.h>

using namespace casadi;



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

//PID controller variables
double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 1;

int proximity_radius_shuttle = 1;
int proximity_radius_target = 15;
int target_shuttle_vertical_closeness = 6;
int target_shuttle_horizontal_closeness = 5;
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

int finished_mission = 0;


int mpc_is_active = 0;


//hardcoded trajectory points
double shuttle_waiting_point[3][1];
double target_inform_point[3][1];
double shuttle_stop_area[3][2];

double referential_relative_to_shuttle_x;
double referential_relative_to_shuttle_y;
double referential_relative_to_shuttle_z;



ros::Timer capture_timer;
ros::Timer drop_timer;


int lala = 0;
DM xx = DM::zeros(14,26);
DM uu = DM::zeros(4,25);

int lele = 0;


double minimum_distance = 10000;


void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);
    desiredHeading= 0; //heading is only relevant when the MPC is activated
}

void calculateDesiredVelocityLinear(){
    double e_pX = desiredPosition.x  - shuttle->ekf.pos[0][0];
    double e_pY = desiredPosition.y  - shuttle->ekf.pos[1][0];
    double e_pZ = desiredPosition.z  - shuttle->ekf.pos[2][0];

    double Tatual = ros::Time::now().toSec();
    position_error[0][0] += (Tatual - Tprev)*e_pX;
    position_error[1][0] += (Tatual - Tprev)*e_pY;
    position_error[2][0] += (Tatual - Tprev)*e_pZ;
    
    double e_dX = (e_pX  - previous_error[0][0])/(Tatual - Tprev);
    double e_dY = (e_pY  - previous_error[1][0])/(Tatual - Tprev);
    double e_dZ = (e_pZ  - previous_error[2][0])/(Tatual - Tprev);

    Tprev = Tatual;
    previous_error[0][0]= e_pX; previous_error[1][0]= e_pY; previous_error[2][0]=e_pZ;

    desiredVelocityLinear.x = kp_velocity*e_pX + ki_velocity*position_error[0][0] + kd_velocity*e_dX;
    desiredVelocityLinear.y = kp_velocity*e_pY + ki_velocity*position_error[1][0] + kd_velocity*e_dY;
    desiredVelocityLinear.z = kp_velocity*e_pZ + ki_velocity*position_error[2][0] + kd_velocity*e_dZ;
    
 

            
    calculateDesiredHeading();

}


void captureStatusResult(){
    //Develop here capture sensing mechanism, as of now, all captures succeed (msg.data = 1;)
    ros::Rate rate(1);



    capture_timer.stop();

   ros::Publisher capture_status_pub = nh->advertise<std_msgs::Int32>("cooperative_planning/state_machine/capture_success", 10);
    ros::Publisher mpc_time_pub = nh->advertise<std_msgs::Float32>("cooperative_planning/mpc_computation_time", 10);

    std::cout << "Sending Capture Status " << std::endl;

    
    mpc_is_active = 0;
    
 
    
    double vel[3][1];
    vel[0][0]=0; vel[1][0]=0; vel[2][0]=0;    
    shuttle->set_vel_yaw(vel, 0, 0.001);

    rate.sleep();

    std_msgs::Int32 msg;
    msg.data = 1;
    capture_status_pub.publish(msg); 

    
     std_msgs::Float32 msg1;
    msg1.data = 1111;
    mpc_time_pub.publish(msg1); 


    rate.sleep();
    std::cout << "Capture Status Sent" << std::endl;

}

void timerCb(const ros::TimerEvent& event){
    captureStatusResult();
}

int aux = 0;

void executeCaptureManeuverCb(const std_msgs::Empty::ConstPtr& msg){

    //Develop here the capture maneuver actions, as of now, mpc will automatically change it's relative distance to the target on the z axis

    std::cout << "Executing Capture Maneuver " << std::endl;

    //a timer will be set to simulate a capture maneuver happening, after the timer runs out, the captureStatusResult function determines if the capture was succefull
    capture_timer = nh->createTimer(ros::Duration(7), timerCb);

    capture_timer.start();

    ros::Rate rate(10);
    rate.sleep();

}


void dropTimerCb(const ros::TimerEvent& event){
 ros::Rate rate(1);

    ros::Publisher target_dropped_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_dropped", 10);
     drop_timer.stop();

    rate.sleep();

    std_msgs::Empty msg;
    target_dropped_pub.publish(msg);

    rate.sleep();
}


void dropTargetCb(const std_msgs::Empty::ConstPtr& msg){

    //Develop here the drop target actions, as of now, this function only sets a timer
    std::cout << "Dropping Target " << std::endl;


    drop_timer = nh->createTimer(ros::Duration(3), dropTimerCb);

    drop_timer.start();
    ros::Rate rate(1);
    rate.sleep();

    

}

void updateDesiredPosCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova posicao
        //rostopic pub /cooperative_planning/state_machine/desired_local_position geometry_msgs/Point  '{x: 10.0, y: 10.0, z: -10.0}'

    desiredPosition.x = msg->x;
    desiredPosition.y = msg->y;
    desiredPosition.z = msg->z;
    mpc_is_active = 0;

}

void updateDesiredVelCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova velocidade
        //rostopic pub /cooperative_planning/state_machine/desired_local_velocity geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

   
    desiredVelocityLinear.x = msg->x;
    desiredVelocityLinear.y = msg->y;
    desiredVelocityLinear.z = msg->z;
}


void updateDesiredPositionFollower(double x, double y, double z){
       
    desiredPosition.x = target->ekf.pos[0][0] + x;
    desiredPosition.y = target->ekf.pos[1][0] + y;
    //desiredPosition.z = target->ekf.pos[2][0] - verticalDistance + z;
    desiredPosition.z = target->ekf.pos[2][0]  + z;
    
}     



void desiredPosReached(ros::Publisher pub){
       
    if ( (abs(shuttle->ekf.pos[0][0]- desiredPosition.x) < proximity_radius_shuttle) && (abs(shuttle->ekf.pos[1][0]- desiredPosition.y) < proximity_radius_shuttle) && (abs(shuttle->ekf.pos[2][0]- desiredPosition.z) < proximity_radius_shuttle) )
        targetReached = 1;
    else{
        targetReached = 0;
        reachedFlag = 0;
    }  
        

    if (targetReached && reachedFlag == 0){
        reachedFlag = 1;
        std_msgs::Empty msg;
        pub.publish(msg);

    }
         
    
}     

void stopAreaReached(ros::Publisher pub){
       


   if(shuttle->ekf.pos[1][0] >= ( (( shuttle_stop_area[1][1] - shuttle_stop_area[1][0])/ (shuttle_stop_area[0][1] - shuttle_stop_area[0][0]))* (shuttle->ekf.pos[0][0] - shuttle_stop_area[0][0]) + shuttle_stop_area[1][0])) 
        targetReached = 1;
    else{
        targetReached = 0;
        reachedFlag = 0;
    }  
        

    if (targetReached && reachedFlag == 0){
        reachedFlag = 1;
        std_msgs::Empty msg;
        pub.publish(msg);

    }
         
    
}     



void targetDesiredPosReached(ros::Publisher pub, double x, double y, double z){
       
    if ( (abs(target->ekf.pos[0][0] + x- target_inform_point[0][0]) < proximity_radius_target) && (abs(target->ekf.pos[1][0] + y - target_inform_point[1][0]) < proximity_radius_target) )
        informPointReached = 1;
    else{
        informPointReached = 0;
        reachedFlagTarget = 0;
    }  
        

    if (informPointReached && reachedFlagTarget == 0){
        reachedFlagTarget = 1;
        std_msgs::Empty msg;
        pub.publish(msg);
        

    }
         
    
}     
void activateMPCCb(const std_msgs::Empty::ConstPtr& msg){
     mpc_is_active = 1;
}
  

void targetIsCloseWarning(ros::Publisher pub, double x, double y, double z){
       
    if ( (abs(target->ekf.pos[0][0] + x- shuttle->ekf.pos[0][0]) < target_shuttle_horizontal_closeness) && (abs(target->ekf.pos[1][0] + y- shuttle->ekf.pos[1][0]) < target_shuttle_horizontal_closeness) )//&& (abs(target->ekf.pos[2][0] + z - shuttle->ekf.pos[2][0]) < target_shuttle_vertical_closeness) )
        targetReachedShuttle = 1;
    else{
        targetReachedShuttle = 0;
        targetCloseFlag = 0;
    }  
        

    if (targetReachedShuttle && targetCloseFlag == 0){
        targetCloseFlag = 1;
        std_msgs::Empty msg;
        pub.publish(msg);



       
    }
         
    
}     


void performLandingCb(const std_msgs::Empty::ConstPtr& msg){
  
    finished_mission = 1;
}







void mpcController(ros::Publisher pub, Function mpc,double relative_x, double relative_y, double relative_z){
       
    
    std::vector<double> xx0 = {  shuttle->ekf.pos[0][0] ,shuttle->ekf.pos[1][0],shuttle->ekf.pos[2][0],  shuttle->ekf.vel[0][0],shuttle->ekf.vel[1][0],shuttle->ekf.vel[2][0],  shuttle->ekf.att_euler[3][0],     target->ekf.pos[0][0] + relative_x ,target->ekf.pos[1][0] + relative_y,target->ekf.pos[2][0] + relative_z ,  target->ekf.vel[0][0],target->ekf.vel[1][0],target->ekf.vel[2][0],  target->ekf.att_euler[3][0] /*+ 1.3962634*/ }; //shuttle and target states for mpc
    
    ros::Time start_time = ros::Time::now();

    /*double dZ = 2;

    if(captureMoment){
        dZ = 0;
        if(!lele){
            xx = DM::zeros(14,26);
        uu = DM::zeros(4,25);
        lele = 1;
        
         }
    } */
    
    
    //std::vector<DM> arg1 ={DM(xx0),dZ, xx, uu};
    std::vector<DM> arg1 ={DM(xx0), xx, uu};

        std::vector<DM> res = mpc(arg1);
    casadi::Matrix<double> result_xx = res.at(1);
    casadi::Matrix<double> result_uu = res.at(0);

        xx = result_xx;
        uu = result_uu;
        


    
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    std::cout << "MPC computation time:  " << delta_t_sec << std::endl;
    //std::cout << "result uu: " << res.at(0) << std::endl;
    //std::cout << "result xx: " << res.at(1) << std::endl;
    
    casadi::Matrix<double> result_matrix = res.at(1);
    //std::cout << "result first element: " << result_matrix(0,0) << std::endl;


    //predicted states for velocity and psi in next instance
    double vel_x = (double)result_matrix(3,1);
    double vel_y = (double)result_matrix(4,1);
    double vel_z = (double)result_matrix(5,1);
    //double psi = (double)result_matrix(6,1);
    double psi = target->ekf.att_euler[3][0] + 1.3962634 ;

    double vel_ned[3][1];
    vel_ned[0][0] = vel_x;
    vel_ned[1][0] = vel_y;
    vel_ned[2][0] = vel_z;

    //if(captureMoment) vel_ned[2][0] = vel_z + 2;





    shuttle->set_vel_yaw(vel_ned, psi, 0.001);

    
    std_msgs::Float32 msg;
    msg.data = (float)delta_t_sec;
    pub.publish(msg); 

    

         
  
         
    
}     



int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_follower_mpc_controller");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

   /* Get the namespace of the drones and other parameters */
    shuttle_drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespaceShuttle");
    shuttle_drone_info.ID = DroneGimmicks::getParameters<double>(*nh, "IDShuttle");
    shuttle_drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    shuttle_drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    shuttle_drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    shuttle_drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    shuttle_drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    shuttle_drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 

    target_drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespaceTarget");
    shuttle_drone_info.ID = DroneGimmicks::getParameters<double>(*nh, "IDTarget");
    target_drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    target_drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    target_drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    target_drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    target_drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    target_drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 


    /* Create the drone objects */
    shuttle = new DroneLib::UAV(shuttle_drone_info.drone_ns, 
        shuttle_drone_info.mass, 
        shuttle_drone_info.radius, 
        shuttle_drone_info.height, 
        shuttle_drone_info.num_rotors, 
        shuttle_drone_info.thrust_curve,
        nh, nh_p); 

    target = new DroneLib::UAV(target_drone_info.drone_ns, 
        target_drone_info.mass, 
        target_drone_info.radius, 
        target_drone_info.height, 
        target_drone_info.num_rotors, 
        target_drone_info.thrust_curve,
        nh, nh_p); 


    
    
  /*  double aux_ned[3][1], aux_enu[3][1];
    aux_enu[0][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceX"); 
    aux_enu[1][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceY"); 
    aux_enu[2][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceZ");
    
    DroneLib::enu_to_ned(aux_enu, aux_ned);
    referential_relative_to_shuttle_x = aux_ned[0][0];
    referential_relative_to_shuttle_y = aux_ned[1][0];
    referential_relative_to_shuttle_z = aux_ned[2][0];*/



    
    referential_relative_to_shuttle_x = 5;
    referential_relative_to_shuttle_y = 5;
    referential_relative_to_shuttle_z = 0;



/*        ROS_WARN_STREAM("Shuttle Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Target Current Position: " << target->ekf.pos[0][0]   << "  " << target->ekf.pos[1][0]  << "  " << target->ekf.pos[2][0]  );
        ROS_WARN_STREAM("Target Current Position COM RELATIVE: " << target->ekf.pos[0][0] + referential_relative_to_shuttle_x  << "  " << target->ekf.pos[1][0] + referential_relative_to_shuttle_y << "  " << target->ekf.pos[2][0] + referential_relative_to_shuttle_z );
*/


     //hardcoded trajectory points
   // shuttle_waiting_point[0][0]=42; shuttle_waiting_point[1][0]=-13; shuttle_waiting_point[2][0]=-25;    
    target_inform_point[0][0]=73; target_inform_point[1][0]=-24; target_inform_point[2][0]=-25;    
    //target_inform_point[0][0]=100; target_inform_point[1][0]=25; target_inform_point[2][0]=-25;    
    //shuttle_stop_area[0][0]=-90; shuttle_stop_area[1][0]=37.33; shuttle_stop_area[2][0]=-25;    
    
    shuttle_stop_area[0][0]=-93; shuttle_stop_area[1][0]=15; shuttle_stop_area[2][0]=-25;    
    shuttle_stop_area[0][1]=-70; shuttle_stop_area[1][1]=100; shuttle_stop_area[2][1]=-25;    





    ros::Subscriber desired_pos_sub = nh->subscribe("cooperative_planning/state_machine/desired_local_position", 10, updateDesiredPosCb);
    ros::Subscriber desired_vel_sub = nh->subscribe("cooperative_planning/state_machine/desired_local_velocity", 10, updateDesiredVelCb);
    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/shuttle_reached_desired_position", 1);
    ros::Publisher target_reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_reached_inform_point", 10);
    ros::Publisher target_is_close_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_is_close", 10);
    ros::Subscriber execute_capture_maneuver_sub = nh->subscribe("cooperative_planning/state_machine/execute_capture_maneuver", 10, executeCaptureManeuverCb);
    //ros::Publisher capture_status_pub = nh->advertise<std_msgs::Int32>("cooperative_planning/state_machine/capture_success", 10);
    ros::Subscriber perform_landing_sub = nh->subscribe("/cooperative_planning/state_machine/perform_landing", 10, performLandingCb);

    ros::Publisher takeoff_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/takeoff", 10);
    ros::Publisher stop_area_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/shuttle_reached_stop_area", 10);
    ros::Subscriber activate_mpc_sub = nh->subscribe("/cooperative_planning/state_machine/activate_mpc", 10, activateMPCCb);

    ros::Subscriber drop_target_sub = nh->subscribe("/cooperative_planning/state_machine/execute_drop_target", 10, dropTargetCb);
    ros::Publisher mission_finished_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/mission_finished", 10);



    ros::Publisher mpc_time_pub = nh->advertise<std_msgs::Float32>("cooperative_planning/mpc_computation_time", 10);
    
    
    
     // file name
    std::string file_name = "gen";
     // code predix
    std::string prefix_code = ros::package::getPath("cooperative_planning") + "/include/";
    // shared library prefix
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";

    // Create a new NLP solver instance from the compiled code
    std::string lib_name = prefix_lib + file_name + ".so";
    // Use CasADi's "external" to load the compiled function

    Function mpc_control = external("F",lib_name);

    
    
    ros::Rate rate(100.0);
    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE BASED ON STATE MACHINE");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;


    
    pos[0][0]= 0; //target->ekf.pos[0][0] + referential_relative_to_shuttle_x;
    pos[1][0] = 0; //target->ekf.pos[1][0] + referential_relative_to_shuttle_y;
    pos[2][0]= -25.0;

   // pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();

    shuttle->set_pos_yaw(pos, yaw, 10);

    std_msgs::Empty msg_takeoff;
    takeoff_pub.publish(msg_takeoff);

    
    Tprev = ros::Time::now().toSec();
    position_error[0][0]= 0; position_error[1][0]= 0; position_error[2][0]=0;
    previous_error[0][0]= 0; previous_error[1][0]= 0; previous_error[2][0]=0;

    while(ros::ok() && !finished_mission){
      
        //ros::Time start_time_main = ros::Time::now();


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
/*        ROS_WARN_STREAM("Shuttle Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Shuttle Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);
        ROS_WARN_STREAM("Shuttle Current Velocity: " << shuttle->ekf.vel[0][0]  << "  " << shuttle->ekf.vel[1][0]  << "  " << shuttle->ekf.vel[2][0]);
        ROS_WARN_STREAM("Shuttle Desired Velocity: " << desiredVelocityLinear.x  << "  " << desiredVelocityLinear.y  << "  " << desiredVelocityLinear.z);

        ROS_WARN_STREAM("Target Current Position: " << target->ekf.pos[0][0] + referential_relative_to_shuttle_x  << "  " << target->ekf.pos[1][0] + referential_relative_to_shuttle_y << "  " << target->ekf.pos[2][0] + referential_relative_to_shuttle_z );
*/

/*double minimum_distance_aux = sqrt( pow(2,(shuttle->ekf.pos[0][0] - (target->ekf.pos[0][0] + referential_relative_to_shuttle_x) ) )  + pow(2,(shuttle->ekf.pos[1][0] -(target->ekf.pos[1][0] + referential_relative_to_shuttle_y )))  + pow(2,(shuttle->ekf.pos[2][0]-( target->ekf.pos[2][0] + referential_relative_to_shuttle_z ))) );
       if( minimum_distance_aux < minimum_distance)
            minimum_distance = minimum_distance_aux;
        ROS_WARN_STREAM("minimum distance between drones " << minimum_distance);
*/


       /*ROS_WARN_STREAM("Shuttle Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
       ROS_WARN_STREAM("Target Position : " << target->ekf.pos[0][0] + referential_relative_to_shuttle_x << "  " << target->ekf.pos[1][0] +referential_relative_to_shuttle_y << "  " << target->ekf.pos[2][0] + referential_relative_to_shuttle_z);
        ROS_WARN_STREAM("Shuttle Velocity: " << shuttle->ekf.vel[0][0]  << "  " << shuttle->ekf.vel[1][0]  << "  " << shuttle->ekf.vel[2][0]);
        ROS_WARN_STREAM("Target Velocity : " << target->ekf.vel[0][0]  << "  " << target->ekf.vel[1][0]  << "  " << target->ekf.vel[2][0] );
        ROS_WARN_STREAM("Shuttle Yaw: " << shuttle->ekf.att_euler[3][0] );
        ROS_WARN_STREAM("Target Yaw : " << target->ekf.att_euler[3][0] + 1.3962634  );*/


        desiredPosReached(reached_pos_pub);
        stopAreaReached(stop_area_pub);
        targetDesiredPosReached(target_reached_pos_pub, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);
        targetIsCloseWarning(target_is_close_pub, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);
        


        if(mpc_is_active==0){
        //if(!captureMoment)
            calculateDesiredVelocityLinear();
        
        /*if(captureStatus){
            captureStatusResult(capture_status_pub);
            captureStatus = 0;
            captureMoment = 0;
            
        }*/

        //if(captureMoment) updateDesiredPositionFollower(referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);


        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        shuttle->set_vel_yaw(vel, desiredHeading, 0.001);
        //REVEER O CONTROLADOR LA DE CIMA PRA CORRIGIr os ganhos


        //pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
        //shuttle->set_pos_yaw(pos, yaw, 0.01);
        
       


        }
        else{

                   mpcController(mpc_time_pub, mpc_control, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);

        }
        


        //ros::Duration delta_t_main = ros::Time::now() - start_time_main;
        //double delta_t_sec_main = delta_t_main.toSec();
        //std::cout << "Main loop computation time:  " << delta_t_sec_main << std::endl;
        

        ros::spinOnce();
        rate.sleep();
    }



    pos[0][0]= -5; 
    pos[1][0] = -5;
    pos[2][0]= -5;
	yaw = 0.0;
    shuttle->set_pos_yaw(pos, yaw, 5);

    shuttle->auto_land();
    
    std_msgs::Empty msg_finished;
    mission_finished_pub.publish(msg_finished);

    ros::Rate end(10);
    end.sleep();

    return 0;


























}
