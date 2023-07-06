#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>

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

double kp_velocity = 5, ki_velocity = kp_velocity/4, kp_heading = 1;

int proximity_radius = 1;
int targetReached = 0;
int reachedFlag = 0;


void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){
    /*desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - shuttle->ekf.pos[0][0]);
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - shuttle->ekf.pos[1][0]);
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - shuttle->ekf.pos[2][0]);*/

    desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - shuttle->ekf.pos[0][0]) + ki_velocity*((desiredPosition.x  - shuttle->ekf.pos[0][0]) - shuttle->ekf.vel[0][0]) ;
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - shuttle->ekf.pos[1][0]) + ki_velocity*((desiredPosition.y  - shuttle->ekf.pos[1][0]) - shuttle->ekf.vel[1][0]) ;
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - shuttle->ekf.pos[2][0]) + ki_velocity*((desiredPosition.z  - shuttle->ekf.pos[2][0]) - shuttle->ekf.vel[2][0]) ;
    calculateDesiredHeading();
}


void updateDesiredPosCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova posicao
        //rostopic pub /cooperative_planning/shuttleController/desired_local_position geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

    desiredPosition.x = msg->x;
    desiredPosition.y = msg->y;
    desiredPosition.z = msg->z;
}


void desiredPosReached(ros::Publisher pub){
       
    if ( (abs(shuttle->ekf.pos[0][0]- desiredPosition.x) < proximity_radius) && (abs(shuttle->ekf.pos[1][0]- desiredPosition.y) < proximity_radius) && (abs(shuttle->ekf.pos[2][0]- desiredPosition.z) < proximity_radius) )
        targetReached = 1;
    else{
        targetReached = 0;
        reachedFlag = 0;
    }  
        

    if (targetReached && reachedFlag == 0){
        reachedFlag = 1;
        std_msgs::Int64 msg;
        msg.data = 1;
        pub.publish(msg);

    }
         
    
}     






int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_vel_control");
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





    //---------------------------------------------------------------

    ros::Subscriber desired_pos_sub = nh->subscribe("cooperative_planning/shuttleController/desired_local_position", 10, updateDesiredPosCb);
    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Int64>("cooperative_planning/shuttleController/reached_target_pos", 10);


    ros::Rate rate(20.0);

    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE ");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -3.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();

    shuttle->set_pos_yaw(pos, yaw, 10);

    
    

    while(ros::ok()){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        desiredPosReached(reached_pos_pub);


        calculateDesiredVelocityLinear();

        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        shuttle->set_vel_yaw(vel, desiredHeading, 0.01);

        //pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
        //shuttle->set_pos_yaw(pos, yaw, 0.01);








        ros::spinOnce();
        rate.sleep();
    }

    return 0;





















    /*// Telemetry test
    int t = 0;
    while (t < 10){
	ROS_WARN_STREAM(shuttle->sen.imu.ang_vel[0][0] << shuttle->sen.imu.ang_vel[1][0] << shuttle->sen.imu.ang_vel[2][0]);
        ROS_WARN_STREAM(shuttle->sen.gps.pos[0][0] << "   " << shuttle->sen.gps.pos[1][0] << "   " << shuttle->sen.gps.pos[2][0]);
	ros::Duration(1).sleep();
	t += 1;
    }


    // Arming test
    shuttle->arm_drone();
    ros::Duration(5).sleep();
    shuttle->disarm_drone();

     // Telemetry test
     t = 0;
    while (t < 10){
	ROS_WARN_STREAM(target->sen.imu.ang_vel[0][0] << target->sen.imu.ang_vel[1][0] << target->sen.imu.ang_vel[2][0]);
        ROS_WARN_STREAM(target->sen.gps.pos[0][0] << "   " << target->sen.gps.pos[1][0] << "   " << target->sen.gps.pos[2][0]);
	ros::Duration(1).sleep();
	t += 1;
    }


    // Arming test
    target->arm_drone();
    ros::Duration(5).sleep();
    target->disarm_drone();

    return 0;*/




}
