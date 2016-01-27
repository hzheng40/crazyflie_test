#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

void publish(double, double, double, double);
void imuCallback(const sensor_msgs::Imu &_msg);

ros::Publisher pub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "crazyflie_test");

	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber imu = node.subscribe("imu", 1, imuCallback);
	ros::Rate loop_rate(50);

	while (ros::ok()) {
		ros::spinOnce();
	}
	return 0;
}

void publish(double pitch, double roll, double yaw, double thrust) {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = pitch;
	cmd_vel.linear.y = roll;
	cmd_vel.linear.z = thrust;
	cmd_vel.angular.z = yaw;
	pub.publish(cmd_vel);
	return;
}
void imuCallback(const sensor_msgs::Imu &_msg) {
	publish(0,0,0,12000);
}