#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/RollPitchYawrateThrust.h>

//#include <sensor_fusion_comm/ExtState.h>

#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <rotors_exercise/ControllerConfig.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define PI 3.14159265
#define GRAVITY 9.81;

//Period for the control loop
float control_loop_period = 0.05;
//Elapsed time between pose messages
float 		delta_time_pose 	  = 0.0;
ros::Time	latest_pose_update_time;

// Feedbacks
sensor_msgs::Imu 			latest_imu;
geometry_msgs::PoseStamped	latest_pose;
nav_msgs::Path				latest_trajectory;
int 						current_index;

// Setpoints
tf::Vector3					setpoint_pos;
double						setpoint_yaw;

// Gravity



// Velocity commands and limits
float x_vel_cmd, y_vel_cmd, z_vel_cmd;
float maxXVel, maxYVel, maxZVel;

// Thrust and torques commands and limits
float x_cmd, y_cmd;
float roll_cmd, pitch_cmd, thrust, yaw_rate;
float maxRoll, maxPitch, maxThrust, maxYaw;

// Acceleration feedback for feedforward
tf::Vector3		body_accel;

//PID Variables
double latest_yaw = 0.0;
double latest;

double  x_kp, x_ki, x_kd, y_kp, y_ki, y_kd, z_kp, z_ki, z_kd, yaw_kp, yaw_ki,
			yaw_kd, roll_limit, pitch_limit, yaw_limit, thrust_limit;

Eigen::Vector4d error, previous_error, integral_error_acumulator, integral_limits(0.5,0.5,0.5,0.5), delta_error;


void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
	ROS_INFO_ONCE("First Imu msg received ");
	latest_imu = *msg; // Handle IMU data.
}
void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	ROS_INFO_ONCE("First Pose msg received ");
	latest_pose = *msg; 	// Handle pose measurements.
}

void MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.

  latest_trajectory.poses.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  ROS_INFO("New trajectory");

  for (size_t i = 0; i < n_commands; ++i) {

    geometry_msgs::PoseStamped wp;
    wp.pose.position.x    = msg->points[i].transforms[0].translation.x;
    wp.pose.position.y    = msg->points[i].transforms[0].translation.y;
    wp.pose.position.z    = msg->points[i].transforms[0].translation.z;
    wp.pose.orientation = msg->points[i].transforms[0].rotation;

    latest_trajectory.poses.push_back(wp);

    ROS_INFO ("WP %d\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t", (int)i,
    	wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, tf::getYaw(wp.pose.orientation));
  }
  current_index = 0;
}

/// Dynamic reconfigureCallback
void reconfigure_callback(rotors_exercise::ControllerConfig &config, uint32_t level)
{
	// Copy new configuration
	//m_config = config;

	x_kp = config.x_kp;
	x_ki= config.x_ki;
	x_kd = config.x_kd;

	y_kp = config.y_kp;
	y_ki = config.y_ki;
	y_kd = config.y_kd;

	z_kp = config.z_kp;
	z_ki = config.z_ki;
	z_kd = config.z_kd;

	yaw_kp = config.yaw_kp;
	yaw_ki = config.yaw_ki;
	yaw_kd = config.yaw_kd;

	roll_limit = config.roll_limit;
	pitch_limit = config.pitch_limit;
	thrust_limit = config.thrust_limit;
	yaw_limit = config.yaw_limit;

	ROS_INFO (" ");
	ROS_INFO ("Reconfigure callback have been called with new Settings ");

}

void timerCallback(const ros::TimerEvent& e)
{
	double roll, pitch, yaw;
	if (latest_pose.header.stamp.nsec > 0.0)
	{
		ROS_INFO ("///////////////////////////////////////");

		// ADD here any debugging you need
		ROS_INFO_STREAM ("SETPOINT POSITION: X: " << setpoint_pos[0] << " Y: " << setpoint_pos[1] << " Z: "<< setpoint_pos[2] << " YAW: " << setpoint_yaw );
		ROS_INFO_STREAM ("CURRENT POSITION:  X: " << latest_pose.pose.position.x << " Y: " << latest_pose.pose.position.y << " Z: "<< latest_pose.pose.position.z << " YAW: " << latest_yaw );
		ROS_INFO_STREAM ("Errors: X: " << error[0] << " Y: " << error[1] << " Z: "<< error[2] << " YAW: " << error[3] );
		ROS_INFO_STREAM ("R-P-Y-T " << roll_cmd << " " << pitch_cmd << " " << yaw_rate << " " << thrust);

	}
}

tf::Vector3 rotateZ (tf::Vector3 input_vector, float angle)
{
	tf::Quaternion quat;
	quat.setRPY(0.0, 0.0, angle);
	tf::Transform transform (quat);

	return (transform * input_vector);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::NodeHandle nh_params("~");

	ROS_INFO("running controller");

	ros::Subscriber imu_sub   = nh.subscribe("imu",  1, &imuCallback);   // remap "/sensor_msgs/Imu"
	ros::Subscriber pose_sub  = nh.subscribe("pose", 1, &poseCallback);  // remap "/ardrone/optitrack_pose" for ardrone

	ros::Subscriber traj_sub  = nh.subscribe("command/trajectory", 1, &MultiDofJointTrajectoryCallback);
	current_index = 0;



	nh_params.param("x_kp", x_kp, 0.0);
	nh_params.param("x_ki", x_ki, 0.0);
	nh_params.param("x_kd", x_kd, 0.0);

	nh_params.param("y_kp", y_kp, 0.0);
	nh_params.param("y_ki", y_ki, 0.0);
	nh_params.param("y_kd", y_kd, 0.0);

	nh_params.param("z_kp", z_kp, 0.0);
	nh_params.param("z_ki", z_ki, 0.0);
	nh_params.param("z_kd", z_kd, 0.0);

	nh_params.param("yaw_kp", yaw_kp, 0.0);
	nh_params.param("yaw_ki", yaw_ki, 0.0);
	nh_params.param("yaw_kd", yaw_kd, 0.0);


	nh_params.param("roll_limit", roll_limit, 0.0);
	nh_params.param("pitch_limit", pitch_limit, 0.0);
	nh_params.param("thrust_limit", thrust_limit, 0.0);
	nh_params.param("yaw_limit", yaw_limit, 0.0);





	// Chose one of the versions below. The first of these topic published determines the control mode.
	ros::Publisher command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
	ros::Publisher ARvelocity_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	// Start the dynamic_reconfigure server
	dynamic_reconfigure::Server<rotors_exercise::ControllerConfig> server;
	dynamic_reconfigure::Server<rotors_exercise::ControllerConfig>::CallbackType f;
  	f = boost::bind(&reconfigure_callback, _1, _2);
  	server.setCallback(f);


	ROS_INFO("Initializing controller ... ");
	/*
		Initialize here your controllers
	*/
	ros::Timer timer;
	timer = nh.createTimer(ros::Duration(0.2), timerCallback);  //Timer for debugging

	// Run the control loop and Fly to x=0m y=0m z=1m
	ROS_INFO("Going to starting position [0,0,1] ...");
	//positionLoop.setPoint(0.0, 0.0, 1.0, 0.0);
	setpoint_pos = tf::Vector3(0.,0.,1.);
	setpoint_yaw = 0.0;

	latest_pose_update_time = ros::Time::now();

	while(ros::ok())
	{
		ros::spinOnce();

		delta_time_pose = (latest_pose.header.stamp - latest_pose_update_time).toSec() ;

		// Check if pose/imu/state data was received
		if (
			(latest_pose.header.stamp.nsec > 0.0)
			&&
			((latest_pose.header.stamp - latest_pose_update_time).toSec() > 0.0)
		   )
		{
			latest_pose_update_time = latest_pose.header.stamp;

			//compute distance to next waypoint
			double distance = sqrt((setpoint_pos[0]-latest_pose.pose.position.x) * (setpoint_pos[0]-latest_pose.pose.position.x) +
							  (setpoint_pos[1]-latest_pose.pose.position.y) * (setpoint_pos[1]-latest_pose.pose.position.y) +
							  (setpoint_pos[2]-latest_pose.pose.position.z) * (setpoint_pos[2]-latest_pose.pose.position.z) );
			if (distance < 0.5)

			{
				//there is still waypoints
				if (current_index < latest_trajectory.poses.size())
				{
					ROS_INFO("Waypoint achieved! Moving to next waypoint");
					geometry_msgs::PoseStamped wp;
    				wp = latest_trajectory.poses[current_index];
    				setpoint_pos[0]=wp.pose.position.x;
					setpoint_pos[1]=wp.pose.position.y;
					setpoint_pos[2]=wp.pose.position.z;
					setpoint_yaw=tf::getYaw(wp.pose.orientation);
					current_index++;
				}else if  (current_index == latest_trajectory.poses.size()) // print once waypoint achieved
				{
					ROS_INFO("Waypoint achieved! No more waypoints. Hovering");
					current_index++;
				}
			}

			//INIT PID
			// run position loop
			double now= (double)ros::Time::now().toSec();
			double delta_time= now -latest;

			latest= now;
			latest_yaw = tf::getYaw(latest_pose.pose.orientation);

			error[0] = setpoint_pos[0] - latest_pose.pose.position.x;
			error[1] = setpoint_pos[1] - latest_pose.pose.position.y;
			error[2] = setpoint_pos[2] - latest_pose.pose.position.z;
			error[3] = setpoint_yaw - latest_yaw;

			if(error[3] > PI){
				error[3]= error[3]-2*PI;
			}else if(error[3] < -PI){
				error[3]= error[3]+2*PI;
			}

			//INTEGRAL
			integral_error_acumulator[0] += (error[0] * delta_time);
			integral_error_acumulator[1] += (error[1] * delta_time);
			integral_error_acumulator[2] += (error[2] * delta_time);
			integral_error_acumulator[3] += (error[3] * delta_time);

			for(int i=0; i < error.rows(); i++){
				if(integral_error_acumulator[i] > integral_limits[i]){
						integral_error_acumulator[i] = integral_limits[i];
				}else if(integral_error_acumulator[i] < -integral_limits[i]){
						integral_error_acumulator[i] = -integral_limits[i];
				}
			}

			//DERIVATIVE
			delta_error[0]=(error[0]-previous_error[0]) / delta_time;
			delta_error[1]=(error[1]-previous_error[1]) / delta_time;
			delta_error[2]=(error[2]-previous_error[2]) / delta_time;
			delta_error[3]=(error [3]-previous_error[3]) / delta_time;

			previous_error[0] = error[0];
			previous_error[1] = error[1];
			previous_error[2] = error[2];
			previous_error[3] = error[3];



			//Get Commands
			x_vel_cmd = (x_kp * error[0]) + (x_ki * integral_error_acumulator[0]) + (x_kd * delta_error[0]);
			y_vel_cmd = (y_kp * error[1]) + (y_ki * integral_error_acumulator[1]) + (y_kd * delta_error[1]);
			z_vel_cmd = (z_kp * error[2]) + (z_ki * integral_error_acumulator[2]) + (z_kd * delta_error[2]);
			yaw_rate = (yaw_kp * error[3]) + (yaw_ki * integral_error_acumulator[3]) + (yaw_kd * delta_error[3]);

			//Rotate TF from WORLD TO BODY FRAME
			tf::Vector3 vector3 (x_vel_cmd , y_vel_cmd, 0.0);
			vector3 = rotateZ (vector3, -tf::getYaw(latest_pose.pose.orientation));
			pitch_cmd = vector3[0];
			roll_cmd = vector3[1];
			thrust = z_vel_cmd;


			//END PID

			// your desired velocities (or accelerations) should be stored in
			// x_vel_cmd,
			// y_vel_cmd,
			// z_vel_cmd,
			// yaw_rate

			// Map velocities (or accelerations) to angles  roll, pitch

			// Saturate your request
			roll_cmd  = (roll_cmd > roll_limit)   ? roll_limit  : ((roll_cmd < -roll_limit)  ? -roll_limit  : roll_cmd);
			pitch_cmd = (pitch_cmd > pitch_limit) ? pitch_limit : ((pitch_cmd < -pitch_limit)? -pitch_limit : pitch_cmd);
			thrust    = (thrust > thrust_limit)   ? thrust_limit: ((thrust < -thrust_limit)  ? -thrust_limit: thrust);
			yaw_rate  = (yaw_rate > yaw_limit)    ? yaw_limit   : ((yaw_rate < -yaw_limit)   ? -yaw_limit   : yaw_rate);


			/////////////////////////////////// PATCH FOR ARDRONE////////////////7///////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////
			// for more info look at http://ardrone-autonomy.readthedocs.io/en/latest/commands.html

			geometry_msgs::Vector3 linear, angular;

			linear.x = pitch_cmd;
			linear.y = roll_cmd;
			linear.z = thrust;

			angular.x =  0.03;
			angular.y = 0.05;
			angular.z = yaw_rate;


			geometry_msgs::Twist ARvelocity;

			ARvelocity.linear = linear;
			ARvelocity.angular = angular;

			ARvelocity_pub.publish(ARvelocity);

			//////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////
			//obtain vertical acceleration
			tf::Quaternion orientation;
			quaternionMsgToTF(latest_pose.pose.orientation, orientation);
			tf::Transform imu_tf = tf::Transform(orientation, tf::Vector3(0,0,0));
			tf::Vector3 imu_accel(latest_imu.linear_acceleration.x,
									latest_imu.linear_acceleration.y,
									latest_imu.linear_acceleration.z);
			body_accel = imu_tf*imu_accel;

			// Send to the attitude controller:
			// roll angle [rad], pitch angle  [rad], thrust [N][rad/s]
			mav_msgs::RollPitchYawrateThrust msg;

			msg.header 	  = latest_pose.header; // use the latest information you have.
			msg.roll 	  = -roll_cmd;
			msg.pitch 	  = pitch_cmd;
			msg.thrust.z  = thrust;
			msg.yaw_rate  = yaw_rate;

			command_pub.publish(msg);

		}

		ros::Duration(control_loop_period).sleep(); // may be set slower.
	}
	return 0;
}
