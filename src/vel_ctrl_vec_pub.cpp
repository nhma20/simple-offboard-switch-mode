#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
#include <deque>


#define PI 3.14159265


using namespace std::chrono_literals;

//creates a VelocityControlVectorAdvertiser class that subclasses the generic rclcpp::Node base class.
class VelocityControlVectorAdvertiser : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		VelocityControlVectorAdvertiser() : Node("vel_ctrl_vect_advertiser") {

			velocity_vector_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
				"vel_ctrl_vect_topic", 10);
						

			lidar_to_mmwave_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/lidar_to_mmwave_pcl",	10,
				std::bind(&VelocityControlVectorAdvertiser::OnDepthMsg, this, std::placeholders::_1));


			odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>( 
				"/fmu/vehicle_odometry/out",	10,
				//std::bind(&VelocityControlVectorAdvertiser::OnOdoMsg, this, std::placeholders::_1));
				[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr _msg) {
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
					} else {
						yaw_deg_ = 360.0 + yaw_ * (180.0/PI); // + because yaw_ is negative
					}
					//RCLCPP_INFO(this->get_logger(), "YAW")
			});

		}

		~VelocityControlVectorAdvertiser() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down vel_ctrl_vect_advertiser..");
		}


	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr velocity_vector_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_subscription_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;  

		int callback_count = 0;
		float yaw_ = 0;
		float yaw_deg_ = 0;
		float yaw_offset_ = 45; // -45
		float cable_dist_ = 1.0;

		void OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
		void VelocityDroneControl(float xv, float yv, float zv);
		float constrain(float val, float lo_lim, float hi_lim);
};


// publish drone velocity vector
void VelocityControlVectorAdvertiser::VelocityDroneControl(float xv, float yv, float zv){

	float vx_NED = 0.0;
	float vy_NED = 0.0;
	float vz_NED = 0.0;

	// translate x and y velocities from local drone frame to global NED frame
	vx_NED += cos((yaw_deg_+yaw_offset_)*(PI/180.0)) * xv;
	vy_NED += sin((yaw_deg_+yaw_offset_)*(PI/180.0)) * xv;
	vx_NED += -sin((yaw_deg_+yaw_offset_)*(PI/180.0)) * yv;
	vy_NED += cos((yaw_deg_+yaw_offset_)*(PI/180.0)) * yv;
	vz_NED += - zv;

	auto vel_ctrl_vect = px4_msgs::msg::TrajectorySetpoint();
	vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	vel_ctrl_vect.x = 0;
	vel_ctrl_vect.y = 0;
	vel_ctrl_vect.z = 0;
	vel_ctrl_vect.yaw = 0;
	vel_ctrl_vect.yawspeed = 0;
	vel_ctrl_vect.vx = vx_NED;
	vel_ctrl_vect.vy = vy_NED;
	vel_ctrl_vect.vz = vz_NED;
	this->velocity_vector_publisher_->publish(vel_ctrl_vect);

}


// mmwave message callback function
void VelocityControlVectorAdvertiser::OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg){

	// read PointCloud2 msg data
	int pcl_size = _msg->width;
	uint8_t *ptr = _msg->data.data();
	const uint32_t  POINT_STEP = 12;
	std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;
	for (size_t i = 0; i < pcl_size; i++)
	{
		pcl_x.push_back(*(reinterpret_cast<float*>(ptr + 0)));
		pcl_y.push_back(*(reinterpret_cast<float*>(ptr + 4)));
		pcl_z.push_back(*(reinterpret_cast<float*>(ptr + 8)));
		ptr += POINT_STEP;
	}

	float closest_dist = std::numeric_limits<float>::max(); 
	float current_dist = 0;
	int closest_dist_idx = 0;
	// find closest point in pointcloud msg
	for (int i = 0; i < pcl_size; i++)
	{
		current_dist = sqrt( pow(pcl_x.at(i), 2) + pow(pcl_y.at(i), 2) + pow(pcl_z.at(i), 2) );
		if( current_dist < closest_dist ){
			closest_dist = current_dist;
			closest_dist_idx = i;
		}
	}

	static float shortest_dist_angle_xz;
	static float shortest_dist_angle_yz;
	static float shortest_dist;
	float prev_shortest_dist_angle_xz;
	float prev_shortest_dist_angle_yz;
	float prev_shortest_dist;
	static std::deque<float> z_filter_q; // fifo queue with random access support
	static std::deque<float> yz_filter_q;
	static std::deque<float> xz_filter_q;
	float z_avg = 0.0;
	float yz_avg = 0.0;
	float xz_avg = 0.0;
	int filter_size = 10;

	if(pcl_size > 0){
		prev_shortest_dist_angle_xz = shortest_dist_angle_xz;
		prev_shortest_dist_angle_yz = shortest_dist_angle_yz;
		prev_shortest_dist = shortest_dist;
		// shortest_dist_angle_xz = asin(pcl_x.at(closest_dist_idx) / sqrt(pow(pcl_x.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		// shortest_dist_angle_yz = asin(pcl_y.at(closest_dist_idx) / sqrt(pow(pcl_y.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		shortest_dist_angle_yz = asin(pcl_x.at(closest_dist_idx) / sqrt(pow(pcl_x.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		shortest_dist_angle_xz = asin(pcl_y.at(closest_dist_idx) / sqrt(pow(pcl_y.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		shortest_dist = closest_dist;
		RCLCPP_INFO(this->get_logger(),  "Dist: %f, \n XZ Angle: %f, \n YZ Angle: %f", shortest_dist, shortest_dist_angle_xz, shortest_dist_angle_yz);
		RCLCPP_INFO(this->get_logger(),  "Yaw: %f", yaw_deg_);





		// Simple average filtering
		/*z_filter_q.push_back(closest_dist);
		yz_filter_q.push_back(shortest_dist_angle_yz);
		xz_filter_q.push_back(shortest_dist_angle_xz);

		while (z_filter_q.size() > filter_size)
		{
			z_filter_q.pop_front();
			yz_filter_q.pop_front();
			xz_filter_q.pop_front();
		}
		
		for (uint64_t i = 0; i < z_filter_q.size(); i++)
		{
			z_avg += z_filter_q[i];
			yz_avg += yz_filter_q[i];
			xz_avg += xz_filter_q[i];
		}
		z_avg = z_avg / z_filter_q.size();
		yz_avg = yz_avg / yz_filter_q.size();
		xz_avg = xz_avg / xz_filter_q.size();
		
		shortest_dist_angle_yz = yz_avg;
		shortest_dist_angle_xz = xz_avg;
		shortest_dist = z_avg;

		RCLCPP_INFO(this->get_logger(),  "Filter size: %d", z_filter_q.size());
		RCLCPP_INFO(this->get_logger(),  "Averages: %f, %f, %f", z_avg, yz_avg, xz_avg);*/





	} else{	
		RCLCPP_INFO(this->get_logger(),  "\n No points in pointcloud");
	}

	float control_distance = cable_dist_;	// desired distance to cable (meters)
	float control_angle = 0.0; 		// desired angle to cable (rad)
	float kp_dist = 0.5; 			// proportional gain for distance controller - nonoise 1.5
	float kp_angle = 5.0; 			// proportional gain for angle controller - nonoise 5.0
	float kd_dist = 0.015; 			// derivative gain for distance controller - nonoise 1.5
	float kd_angle = 0.0005; 		// derivative gain for angle controller - nonoise 0.5
	float ki_dist = 0.01; 			// integral gain for distance controller - nonoise 0.05
	float ki_angle = 0.01; 			// integral gain for angle controller - nonoise 0.05

	float dt = 0.02; // seconds
	float d_dist = ((shortest_dist-control_distance) - (prev_shortest_dist-control_distance)) / dt;
	float d_angle_xz = ((shortest_dist_angle_xz-control_angle) - (prev_shortest_dist_angle_xz-control_angle)) / dt;
	float d_angle_yz = ((shortest_dist_angle_yz-control_angle) - (prev_shortest_dist_angle_yz-control_angle)) / dt;

	static float i_dist;
	static float i_angle_xz;
	static float i_angle_yz;
	i_dist += (shortest_dist-control_distance) * dt;
	// reset integral terms when error crosses 0
	if( (i_dist < 0 && ((shortest_dist-control_distance) * dt > 0)) || (i_dist > 0 && ((shortest_dist-control_distance) * dt < 0)) ){
		i_dist = 0;
	}
	i_angle_xz += (shortest_dist_angle_xz-control_angle) * dt;
	if( (i_angle_xz < 0 && ((shortest_dist_angle_xz-control_angle)* dt > 0)) || (i_angle_xz > 0 && ((shortest_dist_angle_xz-control_angle) * dt < 0)) ){
		i_angle_xz = 0;
	}
	i_angle_yz += (shortest_dist_angle_yz-control_angle) * dt;
	if( (i_angle_yz < 0 && ((shortest_dist_angle_yz-control_angle)* dt > 0)) || (i_angle_yz > 0 && ((shortest_dist_angle_yz-control_angle) * dt < 0)) ){
		i_angle_yz = 0;
	}

	if(pcl_size > 0){
		// VelocityDroneControl(0.0, 0.5, 0.0);
		// return;

		// create velocity control vector to steer drone towards cable
		float x_ctrl_val = - constrain(kp_angle*(shortest_dist_angle_xz-control_angle) + kd_angle*d_angle_xz + ki_angle*i_angle_xz,-1.0,1.0);
		float y_ctrl_val = - constrain(kp_angle*(shortest_dist_angle_yz-control_angle) + kd_angle*d_angle_yz + ki_angle*i_angle_yz,-1.0,1.0);
		float z_ctrl_val = constrain(kp_dist*(shortest_dist-control_distance) + (kd_dist*d_dist) + (ki_dist*i_dist),-0.25,0.25);
		
		VelocityDroneControl(x_ctrl_val, y_ctrl_val, z_ctrl_val);

	} else {	
		// no points from mmwave, hover
		VelocityDroneControl(0.0, 0.0, 0.0);
		RCLCPP_INFO(this->get_logger(), "\n No points to control after, hovering in place.");
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
