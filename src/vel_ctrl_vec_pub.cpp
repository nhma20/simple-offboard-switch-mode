#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h>  
#include <limits>
#include <vector>
#include <string>

#define PI 3.14159265

using namespace std::chrono_literals;

//creates a VelocityControlVectorAdvertiser class that subclasses the generic rclcpp::Node base class.
class VelocityControlVectorAdvertiser : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		VelocityControlVectorAdvertiser() : Node("vel_ctrl_vect_advertiser") {
			publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("vel_ctrl_vect_topic", 10);
						
			odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(  ////
			"/fmu/vehicle_odometry/out",	10,
			std::bind(&VelocityControlVectorAdvertiser::OnOdoMsg, this, std::placeholders::_1));

			auto timer_callback = [this]() -> void {
				VelocityControlVectorAdvertiser::Controller();
			};

			timer_ = this->create_wall_timer(100ms, timer_callback);
		
		}

		~VelocityControlVectorAdvertiser() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down vel_ctrl_vect_advertiser..");
		}


	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;  ////

		float bbox_size_x_;
		float bbox_size_y_;
		float bbox_center_x_;
		float bbox_center_y_;
		float bbox_orientation_;

		float desired_bbox_size_x_ = 50;
		float desired_bbox_center_x_ = 127;

		float yaw_ = 0;
		float yaw_deg_ = 0;

		int callback_count = 0;
		void VelocityDroneControl(float xv, float yawspeed);
		void Controller();
		void OnOdoMsg(const px4_msgs::msg::VehicleOdometry::SharedPtr _msg); ////
		float constrain(float val, float lo_lim, float hi_lim);

};


// publish drone velocity vector
void VelocityControlVectorAdvertiser::VelocityDroneControl(float xv, float yawspeed){

	// translate x velocity from local drone frame to global NED frame
	float x_NED = cos(yaw_deg_*(PI/180.0)) * xv;
	float y_NED = sin(yaw_deg_*(PI/180.0)) * xv;

	auto vel_ctrl_vect = px4_msgs::msg::TrajectorySetpoint();
	vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	vel_ctrl_vect.x = 0;
	vel_ctrl_vect.y = 0;
	vel_ctrl_vect.z = 0;
	vel_ctrl_vect.yaw = 0;
	vel_ctrl_vect.yawspeed = yawspeed;
	vel_ctrl_vect.vx = x_NED;
	vel_ctrl_vect.vy = y_NED;
	vel_ctrl_vect.vz = 0;
	this->publisher_->publish(vel_ctrl_vect);
}


// bounding box message callback function
void VelocityControlVectorAdvertiser::Controller(){

	float distance_error = 10; // positive means move towards
	float x_position_error = 10; // positive means rotate +z

	float kp_dist = 0.01; // proportional gain for distance controller
	float ki_dist = 0.01; // integral gain for distance controller
	//float kd_dist = 0.015; // derivative gain for distance controller

	float kp_x_pos = 0.01; // proportional gain for x position controller
	float ki_x_pos = 0.01; // integral gain for x position controller 
	//float kd_x_pos = 0.0005; // derivative gain for x position controller

	float dt = 0.033; // seconds, approximate time between received msgs
	/*float d_dist = ((shortest_dist-control_distance) - (prev_shortest_dist-control_distance)) / dt;
	float d_angle_xz = ((shortest_dist_angle_xz-control_angle) - (prev_shortest_dist_angle_xz-control_angle)) / dt;
	float d_angle_yz = ((shortest_dist_angle_yz-control_angle) - (prev_shortest_dist_angle_yz-control_angle)) / dt;
	*/

	static float i_dist;
	static float i_x_pos;
	i_dist += (distance_error) * dt;
	// reset integral terms when error crosses 0
	if( (i_dist < 0 && (distance_error * dt > 0)) || (i_dist > 0 && (distance_error * dt < 0)) ){
		i_dist = 0;
	}
	i_x_pos += (x_position_error) * dt;
	if( (i_x_pos < 0 && (x_position_error* dt > 0)) || (i_x_pos > 0 && (x_position_error * dt < 0)) ){
		i_x_pos = 0;
	}

	// create velocity control vector to steer drone towards cable	
	VelocityDroneControl(
		constrain((kp_dist*distance_error) + (ki_dist*i_dist),-1,1), // + (-kd_dist*d_dist)
		constrain(kp_x_pos*(x_position_error) + ki_x_pos*i_x_pos,-0.75,0.75) //+ kd_x_pos*d_angle_yz
		);
}


void VelocityControlVectorAdvertiser::OnOdoMsg(const px4_msgs::msg::VehicleOdometry::SharedPtr _msg){ ////
	// Yaw angle in NED frame:
	//
	//			0
	//
	//	-pi/2	O	pi/2
	//			
	//		pi or -pi
	//
	// Get yaw from quaternion
	yaw_ = atan2(2.0 * (_msg->q[3] * _msg->q[0] + _msg->q[1] * _msg->q[2]) , - 1.0 + 2.0 * (_msg->q[0] * _msg->q[0] + _msg->q[1] * _msg->q[1]));

	// convert to degrees
	if (yaw_ > 0){
		yaw_deg_ = yaw_ * (180.0/PI);
	}
	else {
		yaw_deg_ = 360.0 + yaw_ * (180.0/PI); // + because yaw_ is negative
	}
} 


// constrains value to be between limits
float VelocityControlVectorAdvertiser::constrain(float val, float lo_lim, float hi_lim){
	if (val < lo_lim)
	{
		return lo_lim;
	}
	else if (val > hi_lim)
	{
		return hi_lim;
	}
	else{
		return val;
	}
}

	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting velocity control vector advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityControlVectorAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
