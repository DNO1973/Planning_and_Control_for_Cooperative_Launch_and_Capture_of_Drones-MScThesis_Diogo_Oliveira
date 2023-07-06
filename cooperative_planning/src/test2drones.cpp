#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>


ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * shuttle = nullptr;
DroneLib::UAV * target = nullptr;

DroneLib::DroneInfo shuttle_drone_info;
DroneLib::DroneInfo target_drone_info;


int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "test2drones");
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

    ROS_INFO("STARTING");
    ros::Duration(5).sleep();

    // Telemetry test
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

    return 0;
}
