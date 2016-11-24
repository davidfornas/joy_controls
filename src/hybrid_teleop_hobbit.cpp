#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <mar_robot_arm5e/ARM5Arm.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>

#define CURRENT_THRESHOLD 1.0

class TeleopUWSim {
public:
	TeleopUWSim();

private:

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes);

	ros::NodeHandle nh_;

	int lastValue_[5];
	ros::Publisher vel_pub_, thrust_pub_, cart_pub_, platform_pub_;
	ros::Subscriber joy_sub_, arm_angle_sub_;
	double angles_[5], desired_[5];
	double speed_;
	int loop_;

	enum Modes {
		VehicleDrive, ArmJoint, ArmCartesian, Head
	};
	Modes mode;
	std::string frame_id_;

};

TeleopUWSim::TeleopUWSim() {

	mode = VehicleDrive;
	speed_ = 0.35;
	loop_ = 0;

	vel_pub_ = nh_.advertise<sensor_msgs::JointState>(
			"/uwsim/joint_state_command", 1);
	thrust_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
			"/g500/thrusters_input", 1);
	cart_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
			"/target_velocity_in", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("arm_joy", 10,
			&TeleopUWSim::joyCallback, this);
	arm_angle_sub_ = nh_.subscribe<sensor_msgs::JointState>(
			"/uwsim/joint_state", 1, &TeleopUWSim::armAngleCallback, this);

	platform_pub_ = nh_.advertise<nav_msgs::Odometry>("/dataNavigator",1);

}

void TeleopUWSim::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	sensor_msgs::JointState js;
	// Assign functions
	if (joy->buttons[0] == 1 && lastValue_[0] != 1
			&& mode != VehicleDrive) {
		mode = VehicleDrive;
		ROS_INFO("Button A pressed, Driving vehicle enabled");
	}
	if (joy->buttons[1] == 1 && lastValue_[1] != 1
			&& mode != ArmJoint) {
		mode = ArmJoint;
		ROS_INFO("Button B pressed, Arm joint control enabled");
	}
	if (joy->buttons[2] == 1 && lastValue_[2] != 1
			&& mode != Head) {
		mode = Head;
		ROS_INFO("Button X pressed, Head control enabled");
	}
	double gripper = 0;
	if (joy->buttons[4] == 1){
		gripper += speed_;
		ROS_DEBUG("Gripper open");
	}else if (joy->buttons[5] == 1){
		gripper -= speed_;
		ROS_DEBUG("Gripper close");
	}

	if (mode == ArmJoint) {
		//More joints...
		js.name.push_back(std::string("Joint1"));
		if (joy->axes[0] > 0.2 || joy->axes[0] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[0]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint2"));
		if (joy->axes[1] > 0.2 || joy->axes[1] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[1]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint3"));
		if (joy->axes[3] > 0.2 || joy->axes[3] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[3]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint4"));
		if (joy->axes[4] > 0.2 || joy->axes[4] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[4]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint5"));
		if (joy->axes[6] > 0.2 || joy->axes[6] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[6]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint6"));
		if (joy->axes[7] > 0.2 || joy->axes[7] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[7]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Joint_Jaw1"));
		js.velocity.push_back(gripper);
		js.name.push_back(std::string("Joint_Jaw2"));
		js.velocity.push_back(gripper);

		js.name.push_back(std::string("JointN"));js.velocity.push_back(0);
		js.name.push_back(std::string("JointH"));js.velocity.push_back(0);

		vel_pub_.publish(js);

	} else if (mode == Head) {

		//More joints...
		js.name.push_back(std::string("Joint1"));js.velocity.push_back(0);
		js.name.push_back(std::string("Joint2"));js.velocity.push_back(0);
		js.name.push_back(std::string("Joint3"));js.velocity.push_back(0);
		js.name.push_back(std::string("Joint4"));js.velocity.push_back(0);
		js.name.push_back(std::string("Joint5"));js.velocity.push_back(0);
		js.name.push_back(std::string("Joint6"));js.velocity.push_back(0);

		js.name.push_back(std::string("Joint_Jaw1"));
		js.velocity.push_back(gripper);
		js.name.push_back(std::string("Joint_Jaw2"));
		js.velocity.push_back(gripper);
		js.name.push_back(std::string("JointN"));
		if (joy->axes[0] > 0.2 || joy->axes[0] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[0]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("JointH"));
		if (joy->axes[1] > 0.2 || joy->axes[1] < -0.2) {
			js.velocity.push_back(speed_ * joy->axes[1]);
		} else {
			js.velocity.push_back(0);
		}
		vel_pub_.publish(js);

	} else { //VehicleDrive
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=0.0;
		odom.pose.pose.position.y=0.0;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation.x=0.0;
		odom.pose.pose.orientation.y=0.0;
		odom.pose.pose.orientation.z=0.0;
		odom.pose.pose.orientation.w=1;

		odom.twist.twist.linear.x=0;
		odom.twist.twist.linear.y=-joy->axes[1]*speed_/2.0;
		odom.twist.twist.linear.z=0;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=joy->axes[3]*speed_/2.0;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		platform_pub_.publish(odom);
	}
	//Save the last button status
	lastValue_[0] = joy->buttons[0];
	lastValue_[1] = joy->buttons[1];
	lastValue_[2] = joy->buttons[2];
	lastValue_[3] = joy->buttons[3];


	if (loop_ % 150 == 0){
		ROS_INFO_STREAM("USAGE INSTRUCTIONS" << std::endl
				<< "A: Drive vehicle with the sticks" << std::endl
				<< "B: Move the arm with the sticks" << std::endl
				<< "X: Move the head with the sticks" << std::endl);

	}
	loop_++;
}

void TeleopUWSim::armAngleCallback(
		const sensor_msgs::JointState::ConstPtr& mes) {
	for (int i = 0; i < 5; i++)
		angles_[i] = mes->position[i];
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "hobbit_joy_control");
	TeleopUWSim teleop_uwsim;

	ros::spin();
}



