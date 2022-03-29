/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <thread>
#include <climits>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif
		
		// check nav_state if in offboard (14)
		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
			"/fmu/vehicle_status/out",
			10,
			[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
			arming_state_ = msg->arming_state;
			nav_state_ = msg->nav_state;
			});


		// get common timestamp
		timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out",
		10,
		[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
			timestamp_.store(msg->timestamp);
		});


		// Get velocity vector values
		// TrajectorySetpoint: https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
		vel_ctrl_subscription_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
			"vel_ctrl_vect_topic",	10,
			[this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg) {
					x_ = msg->x;
					y_ = msg->y;
					z_ = msg->z;
					yaw_ = msg->yaw;
					yawspeed_ = msg->yawspeed;
					vx_ = msg->vx;
					vy_ = msg->vy;
					vz_ = msg->vz;
				});


		auto control_callback = [this]() -> void {

			// If drone not armed (from external controller), do nothing
			/*if (nav_state_ != 14) {
				RCLCPP_INFO(this->get_logger(), "nav_state: %d", nav_state_);
				RCLCPP_INFO(this->get_logger(), "Waiting for offboard mode");
				return;
			}*/

			if (offboard_setpoint_counter_ == 1) { // 5s sleep before starting offboard
			RCLCPP_INFO(this->get_logger(), "Offboard mode detected");
				RCLCPP_INFO(this->get_logger(), "Waiting 5 seconds before arming");
				std::chrono::nanoseconds sleepperiod(5000000000); 
				rclcpp::GenericRate<std::chrono::high_resolution_clock> rate(sleepperiod);
				rate.sleep();  
			}


			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
				RCLCPP_INFO(this->get_logger(), "Takeoff and hover");
			}


			// Offboard_control_mode needs to be paired with trajectory_setpoint
			if (offboard_setpoint_counter_ < hover_count_) 
			{
				publish_offboard_control_mode();
				publish_hover_setpoint();
			} 
			else if (offboard_setpoint_counter_ < yaw_count_)
			{
				if (offboard_setpoint_counter_ == hover_count_)
				{
					RCLCPP_INFO(this->get_logger(), "Aligning yaw with cables (approximate)");
				}
				publish_offboard_control_mode();
				publish_yaw_setpoint();
			}
			else if (offboard_setpoint_counter_ < tracking_count_)
			{
				if (offboard_setpoint_counter_ == yaw_count_)
				{
					RCLCPP_INFO(this->get_logger(), "Attempting to reach cable");
				}
				publish_offboard_control_mode();
				publish_trajectory_setpoint();
			} 
			else 
			{
				if (offboard_setpoint_counter_ == tracking_count_)
				{
					RCLCPP_INFO(this->get_logger(), "Landing at current position");
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
				}
			}


			offboard_setpoint_counter_++;

		};


		timer_ = this->create_wall_timer(50ms, control_callback);

	}
	
	~OffboardControl() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control..");
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND); 
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
	}
	
	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vel_ctrl_subscription_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint8_t arming_state_; // armed = 4
	uint8_t nav_state_; // offboard = 14
	uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent
	uint64_t hover_count_ = 200;	// 10s at 20Hz
	uint64_t yaw_count_ = 300; // 5s at 20Hz
	uint64_t tracking_count_ = 900; // 15s at 20Hz (tracking_count_ - yaw_count_ = tracking time)

	float hover_height_ = 1.5;
	float yaw_align_ = -2.0; // -1.15   -0.9

	bool armed = false;
	float x_ = 0, y_ = 0, z_ = 0;
	float yaw_ = 0, yawspeed_ = 0;
	float vx_ = 0, vy_ = 0, vz_ = 0;

	void publish_offboard_control_mode() const;
	void publish_hover_setpoint() const;
	void publish_yaw_setpoint() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0,
				     float param3 = 0.0,
				     float param4 = 0.0,
				     float param5 = 0.0,
				     float param6 = 0.0,
				     float param7 = 0.0) const;
};


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() {
	armed = true;
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() {
	armed = false;
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Vehicle hovers at hover_height_ meters.
 */
void OffboardControl::publish_hover_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = NAN; 		// in meters NED
	msg.y = NAN;
	msg.z = -hover_height_;
	msg.yaw = NAN; 
	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Vehicle yaw with north (approximate).
 */
void OffboardControl::publish_yaw_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = NAN; 		// in meters NED
	msg.y = NAN;
	msg.z = -hover_height_;
	msg.yaw = yaw_align_;
	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = NAN; 		// in meters NED
	msg.y = NAN; 		// in meters NED
	msg.z = NAN; 		// in meters NED
	msg.yaw = NAN; 		// in radians NED -PI..+PI
	msg.yawspeed = 0.0; // in radians/sec
	msg.vx = vx_; 		// in meters/sec
	msg.vy = vy_; 		// in meters/sec
	msg.vz = vz_; 		// in meters/sec
	RCLCPP_INFO(this->get_logger(),  "\n Velocity vector: \n vx: %f, vy: %f, vz: %f'", vx_, vy_, vz_);
	//msg.acceleration	// in meters/sec^2
	//msg.jerk			// in meters/sec^3
	//msg.thrust		// normalized thrust vector in NED

	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2, float param3, float param4,
					      float param5, float param6,
					      float param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
