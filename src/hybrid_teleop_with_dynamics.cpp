#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>

#define CURRENT_THRESHOLD 1.0

class TeleopUWSim {
public:
	TeleopUWSim();

private:

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes);

	//Related with race timing
	void vehiclePoseCallback(const geometry_msgs::Pose::ConstPtr& mes);
	double lastTime_, total_time_;
	bool race_;
	int target_;

	ros::NodeHandle nh_;

	int axisindex[6];
	int AxisDir[5];
	int buttonindex[5], lastValue_[5];
	//double error_;
	double scale_;
	ros::Publisher vel_pub_, thrust_pub_, cart_pub_;
	ros::Subscriber joy_sub_, arm_angle_sub_, veh_pose_sub_;
	double angles_[5], desired_[5];

	enum Modes {
		VehicleDrive, ArmJoint, ArmCartesian
	};
	Modes mode;
	std::string frame_id_;

};

TeleopUWSim::TeleopUWSim() {
	for (int i = 0; i < 5; i++)
		AxisDir[i] = 1;
	mode = VehicleDrive;
	nh_.param("SlewAxis", axisindex[0], axisindex[0]);
	nh_.param("ShoulderAxis", axisindex[1], axisindex[1]);
	nh_.param("ElbowAxis", axisindex[2], axisindex[2]);
	nh_.param("JawRotateAxis", axisindex[3], axisindex[3]);
	nh_.param("JawCloseAxis", axisindex[4], axisindex[4]);
	nh_.param("JawOpenAxis", axisindex[5], axisindex[5]);
	nh_.param("SlewDir", AxisDir[0], AxisDir[0]);
	nh_.param("ShoulderDir", AxisDir[1], AxisDir[1]);
	nh_.param("ElbowDir", AxisDir[2], AxisDir[2]);
	nh_.param("WristDir", AxisDir[3], AxisDir[3]);
	nh_.param("JawDir", AxisDir[4], AxisDir[4]);
	nh_.param("scale", scale_, scale_);
	nh_.param("VehicleModeButton", buttonindex[0], buttonindex[0]);
	nh_.param("JointsModeButton", buttonindex[1], buttonindex[1]);
	nh_.param("CartesianModeButton", buttonindex[2], buttonindex[2]);
	nh_.param("ArmBaseFrameId", frame_id_, frame_id_);

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
	veh_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>(
			"/g500/pose", 1, &TeleopUWSim::vehiclePoseCallback, this);

	race_=false;
	lastTime_ = ros::Time::now().toSec();
	total_time_ = 0;
	target_ = 0;

}

void TeleopUWSim::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	sensor_msgs::JointState js;
	// Assign functions
	if (joy->buttons[buttonindex[0]] == 1 && lastValue_[0] != 1
			&& mode != VehicleDrive) {
		mode = VehicleDrive;
		ROS_INFO("Button %d pressed, Driving vehicle enabled", buttonindex[0]);
	}
	if (joy->buttons[buttonindex[1]] == 1 && lastValue_[1] != 1
			&& mode != ArmJoint) {
		mode = ArmJoint;
		ROS_INFO("Button %d pressed, Arm joint control enabled",
				buttonindex[1]);
	}
	/*
	if (joy->buttons[buttonindex[2]] == 1 && lastValue_[2] != 1
			&& mode != ArmCartesian) {
		mode = ArmCartesian;
		ROS_INFO("Button %d pressed, Arm cartesian enabled", buttonindex[2]);
	}
	*/
	if (joy->buttons[7] == 1 && !race_) {
		race_ = true;
		lastTime_ = ros::Time::now().toSec();
		ROS_INFO("Race started, go towards the purple ring");
	}

	if (mode == ArmJoint) {
		js.name.push_back(std::string("Slew"));
		if (joy->axes[axisindex[0]] > 0.2 || joy->axes[axisindex[0]] < -0.2) {
			js.velocity.push_back(0.2 * joy->axes[axisindex[0]] * AxisDir[0]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Shoulder"));
		if (joy->axes[axisindex[1]] > 0.1 || joy->axes[axisindex[1]] < -0.1) {
			js.velocity.push_back(0.2 * joy->axes[axisindex[1]] * AxisDir[1]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("Elbow"));
		if (joy->axes[axisindex[2]] > 0.1 || joy->axes[axisindex[2]] < -0.1) {
			js.velocity.push_back(0.2 * joy->axes[axisindex[2]] * AxisDir[2]);
		} else {
			js.velocity.push_back(0);
		}
		js.name.push_back(std::string("JawRotate"));
		if (joy->axes[axisindex[3]] > 0.2 || joy->axes[axisindex[3]] < -0.2) {
			js.velocity.push_back(0.3 * joy->axes[axisindex[3]] * AxisDir[3]);
		} else {
			js.velocity.push_back(0);
		}

		js.name.push_back(std::string("JawOpening"));
		if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
			if (joy->axes[axisindex[4]] != 0)

				js.velocity.push_back(
						(joy->axes[axisindex[4]] - 1) * AxisDir[4] * 0.4);
			else
				js.velocity.push_back(0);
		} else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
			if (joy->axes[axisindex[5]] != 0)
				js.velocity.push_back(
						-(joy->axes[axisindex[5]] - 1) * AxisDir[4] * 0.4);
			else
				js.velocity.push_back(0);
		} else {
			js.velocity.push_back(0);
		}
		vel_pub_.publish(js);
	} else if (mode == ArmCartesian) {


		/* KEEP FOR HOBBIT
		float x = 0, y = 0, z = 0;
		if (joy->axes[axisindex[0]] > 0.2 || joy->axes[axisindex[0]] < -0.2)
			x = 0.2 * joy->axes[axisindex[0]];
		if (joy->axes[axisindex[1]] > 0.2 || joy->axes[axisindex[1]] < -0.2)
			z = 0.2 * joy->axes[axisindex[1]];
		if (joy->axes[axisindex[3]] > 0.2 || joy->axes[axisindex[3]] < -0.2)
			y = 0.2 * joy->axes[axisindex[2]];
		geometry_msgs::TwistStamped target_twist;
		target_twist.header.frame_id = frame_id_;
		target_twist.header.stamp = ros::Time::now();
		//Clear velocity output
		target_twist.twist.linear.x = x;
		target_twist.twist.linear.y = y;
		target_twist.twist.linear.z = z;
		target_twist.twist.angular.x = 0.0;
		target_twist.twist.angular.y = 0.0;
		target_twist.twist.angular.z = 0.0;

		cart_pub_.publish(target_twist);
		*/

	} else { //VehicleDrive
		std_msgs::Float64MultiArray thrust;
		//TODO PROPORTIONAL THRUST  to  scale_*joy->axes[axisindex[0]]*AxisDir[0]/-1500;
		//Front thurst
		if (joy->axes[axisindex[1]] > 0.2 || joy->axes[axisindex[1]] < -0.2) {
			thrust.data.push_back(-1.2*joy->axes[axisindex[1]]);
			thrust.data.push_back(-1.2*joy->axes[axisindex[1]]);
		} else {
			thrust.data.push_back(0);
			thrust.data.push_back(0);
		}
		//
		if (joy->axes[axisindex[2]] > 0.2 || joy->axes[axisindex[2]] < -0.2) {
			thrust.data.push_back(1.2*joy->axes[axisindex[2]]);
			thrust.data.push_back(1.2*joy->axes[axisindex[2]]);
		} else {
			thrust.data.push_back(0);
			thrust.data.push_back(0);
		}
		//Side thrust
		if (joy->axes[axisindex[3]] > 0.2 || joy->axes[axisindex[3]] < -0.2) {
			thrust.data.push_back(-1.2*joy->axes[axisindex[3]]);
		} else {
			thrust.data.push_back(0);
		}
		if (joy->axes[axisindex[0]] > 0.4 || joy->axes[axisindex[0]] < -0.4) {
			thrust.data[0] += 1 * joy->axes[axisindex[0]];
			thrust.data[1] -= 1 * joy->axes[axisindex[0]];
		}
		thrust_pub_.publish(thrust);
	}

	//Save the last button status
	lastValue_[0] = joy->buttons[buttonindex[0]];
	lastValue_[1] = joy->buttons[buttonindex[1]];
	lastValue_[2] = joy->buttons[buttonindex[2]];
}

void TeleopUWSim::armAngleCallback(
		const sensor_msgs::JointState::ConstPtr& mes) {
	for (int i = 0; i < 5; i++)
		angles_[i] = mes->position[i];
}

void TeleopUWSim::vehiclePoseCallback(
		const geometry_msgs::Pose::ConstPtr& mes) {
	double now = ros::Time::now().toSec();
	if(race_){
		double currentx = mes->position.x;
		double currenty = mes->position.y;
		double currentz = mes->position.z;
		switch(target_){
		case 0:
			if( currentx > 8.5 && currentx < 11.5 && currenty > -6.5 && currenty < -3.5 && currentz > 8.5 && currentz < 11.5 ){
				total_time_ += (now - lastTime_);
				ROS_ERROR_STREAM("Ring completed in "<< (now - lastTime_) << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;
		case 1:
			if( currentx > 3.5 && currentx < 6.6 && currenty > 13.5 && currenty < 16.5 && currentz > 6.5 && currentz < 9.5 ){
				total_time_ += (now - lastTime_);
				ROS_ERROR_STREAM("Ring completed in "<< (now - lastTime_) << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;
		case 2:
			if( currentx > 13.5 && currentx < 16.5 && currenty > 28.5 && currenty < 31.5 && currentz > 8.5 && currentz < 11.5 ){
				total_time_ += (now - lastTime_);
				ROS_ERROR_STREAM("Ring completed in "<< (now - lastTime_) << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;
		case 3:
			if( currentx > 28.5 && currentx < 31.5 && currenty > 28.5 && currenty < 31.5 && currentz > 8 && currentz < 11.5 ){
				total_time_ += (now - lastTime_);
				ROS_ERROR_STREAM("Ring completed in "<< (now - lastTime_) << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;
		case 4:
			if( currentx > 26.5 && currentx < 29.5 && currenty > 8.5 && currenty < 11.5 && currentz > 1.5 && currentz < 4.5 ){
				total_time_ += (now - lastTime_);
				ROS_ERROR_STREAM("Ring completed in "<< (now - lastTime_) << "seconds. Race finished in " << total_time_ << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;
		/*case 5:
			if( currentx > 8.5 && currentx < 11.5 && currenty > -6.5 && currenty < -3.5 && currentz > 8.5 && currentz < 11.5 ){
				total_time_ += (now - lastTime_);
				ROS_INFO_STREAM("Ring completed in "<< (now - lastTime_) << "seconds. Race finished in " << total_time_ << "seconds.");
				target_ += 1;
				lastTime_ = now;

			}break;*/

		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "arm5_joy_control_with_dynamics");
	TeleopUWSim teleop_uwsim;

	ros::spin();
}

/*  DYNAMICS CONTROL BASED ON UWSIM_DYNAMICS_KEYBOARD
 * #!/usr/bin/env python

 from std_msgs.msg import Float64MultiArray
 import termios, fcntl, sys, os
 import rospy

 #import services
 from std_srvs.srv import Empty

 if len(sys.argv) != 4:
 sys.exit("Usage: "+sys.argv[0]+" <thrusters_topic>")


 thrusters_topic=sys.argv[1]

 fd = sys.stdin.fileno()
 oldterm = termios.tcgetattr(fd)
 newattr = termios.tcgetattr(fd)
 newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
 termios.tcsetattr(fd, termios.TCSANOW, newattr)

 oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
 fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

 pub = rospy.Publisher(thrusters_topic, Float64MultiArray)
 rospy.init_node('keyboard')
 rospy.wait_for_service('/dynamics/reset')
 reset=rospy.ServiceProxy('/dynamics/reset', Empty)
 try:
 while not rospy.is_shutdown():
 thrusters=[0,0,0,0,0]
 msg = Float64MultiArray()
 try:
 c = sys.stdin.read(1)
 ##print "Got character", repr(c)
 if c=='w':
 thrusters[0]=thrusters[1]=0.4
 elif c=='s':
 thrusters[0]=thrusters[1]=-0.4
 elif c=='a':
 thrusters[4]=0.4
 elif c=='d':
 thrusters[4]=-0.4
 elif c==' ':
 reset()
 elif c=='\x1b':
 c2= sys.stdin.read(1)
 c2= sys.stdin.read(1)
 if c2=='A':
 thrusters[2]=thrusters[3]=0.4
 elif c2=='B':
 thrusters[2]=thrusters[3]=-0.4
 elif c2=='C':
 thrusters[0]=-0.4
 thrusters[1]=0.4
 elif c2=='D':
 thrusters[0]=0.4
 thrusters[1]=-0.4
 else:
 print 'wrong key pressed'
 while c!='':
 c = sys.stdin.read(1)
 except IOError: pass
 msg.data = thrusters
 pub.publish(msg)
 rospy.sleep(0.1)
 finally:
 termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
 fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
 */

