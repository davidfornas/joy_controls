#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#define CURRENT_THRESHOLD 1.0

class TeleopUWSim
{
public:
  TeleopUWSim();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes);
  
  ros::NodeHandle nh_;

  int axisindex[6];
  int AxisDir[5];
  int buttonindex[5], lastValue_[5];
  double error_;
  double scale_;
  ros::Publisher vel_pub_, position_pub_;
  ros::Subscriber joy_sub_, arm_angle_sub_;
  bool vehicleMode_;
  double angles_[5], desired_[5];
  
};


TeleopUWSim::TeleopUWSim()
{
  for (int i=0; i<5; i++)
        AxisDir[i]=1;
  vehicleMode_=false;
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
  
  vel_pub_ = nh_.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command",1);
  position_pub_ =nh_.advertise<nav_msgs::Odometry>("/dataNavigator",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("arm_joy", 10, &TeleopUWSim::joyCallback, this);
  arm_angle_sub_= nh_.subscribe<sensor_msgs::JointState>("/uwsim/joint_state", 1, &TeleopUWSim::armAngleCallback, this);

}

void TeleopUWSim::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	sensor_msgs::JointState js;

	if (joy->buttons[buttonindex[0]]==1 && lastValue_[0]!=1){
		vehicleMode_=vehicleMode_?false:true;
		ROS_INFO("Button %d pressed, Driving vehicle: %s", buttonindex[0], vehicleMode_?"yes":"no" );
	}
	
		if (!vehicleMode_){
			js.name.push_back(std::string("Slew"));
			if(joy->axes[axisindex[0]]>0.2 || joy->axes[axisindex[0]]<-0.2){
				js.position.push_back(angles_[0]+joy->axes[axisindex[0]]*AxisDir[0]/6);
			}
			else{
				js.position.push_back(angles_[0]);
			}
			js.name.push_back(std::string("Shoulder"));
			js.position.push_back(angles_[1]+joy->axes[axisindex[1]]*AxisDir[1]/6);

			js.name.push_back(std::string("Elbow"));

			js.position.push_back(angles_[2]-joy->axes[axisindex[2]]*AxisDir[2]/6);

			js.name.push_back(std::string("JawRotate"));

			js.position.push_back(angles_[3]+joy->axes[axisindex[3]]*AxisDir[3]/6);

			js.name.push_back(std::string("JawOpening"));
			if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
				if(joy->axes[axisindex[4]]!=0)
					js.position.push_back(angles_[4]+((joy->axes[axisindex[4]]-1)/4)*AxisDir[4]);
				else
					js.position.push_back(angles_[4]);
			} else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
				if(joy->axes[axisindex[5]]!=0)
					js.position.push_back(angles_[4]-((joy->axes[axisindex[5]]-1)/4)*AxisDir[4]);
				else
					js.position.push_back(angles_[4]);
			} else {
				js.position.push_back(angles_[4]);
			}
			vel_pub_.publish(js);
		}else{
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=0.0;
		odom.pose.pose.position.y=0.0;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation.x=0.0;
		odom.pose.pose.orientation.y=0.0;
		odom.pose.pose.orientation.z=0.0;
		odom.pose.pose.orientation.w=1;

		odom.twist.twist.linear.y=scale_*joy->axes[axisindex[0]]*AxisDir[0]/-1500;
		odom.twist.twist.linear.x=scale_*joy->axes[axisindex[1]]*AxisDir[1]/1500;
		odom.twist.twist.linear.z=scale_*joy->axes[axisindex[2]]*AxisDir[2]/1500;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=scale_*joy->axes[axisindex[3]]*AxisDir[3]/1500;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		position_pub_.publish(odom);
		}
	
	lastValue_[0]=joy->buttons[buttonindex[0]];
}

void TeleopUWSim::armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes){
	for(int i=0;i<5;i++) angles_[i]=mes->position[i];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm5_joy_control");
	TeleopUWSim teleop_uwsim;

	ros::spin();
}

