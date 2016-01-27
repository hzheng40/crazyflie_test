#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

void publish(double, double, double, double);
void imuCallback(const sensor_msgs::Imu &_msg);
void pressureCallback(const std_msgs::Float32::ConstPtr &float_msg);
void batteryCallback(const std_msgs::Float32::ConstPtr &battery_msg);
void genericCallback(double, double, double, double, float);
void gestureCallback(const geometry_msgs::Quaternion &gesture_msg);

ros::Publisher pub;

double kip = 0;
double kdp = 10;
double kpp = 5;
double kir = 0;
double kdr = 10;
double kpr = 6;


// double kip = 0;
// double kdp = 0;
// double kpp = 5;
// double kir = 0;
// double kdr = 0;
// double kpr = 6;

double count = 1;
double delay = 0;

// double kppr = 43;
double kdpr = 100;
double kipr = 0;

int command = 0;

double err_pitchsum, err_rollsum;
float prev_pressure, err_pressure_sum;
bool pressureUpdate, imuUpdate, batteryUpdate;
bool gestureUpdate = 1;
float pressure, battery;
double linear_x,linear_y,angular_x,angular_y;

int main(int argc, char** argv) {
	ros::init(argc, argv, "crazyflie_test");

	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber imu = node.subscribe("imu", 1, imuCallback);
	ros::Subscriber pressure = node.subscribe("pressure", 0, pressureCallback);
	ros::Subscriber gesture = node.subscribe("gesture",1, gestureCallback);
	ros::Subscriber battery = node.subscribe("battery", 1, batteryCallback);

	ros::Rate loop_rate(50);

	while (ros::ok()) {
		ros::spinOnce();
	}
	return 0;
}


void publish(double pitch, double roll, double yawrate, double thrust) {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = pitch;
	cmd_vel.linear.y = roll;
	cmd_vel.linear.z = thrust;
	cmd_vel.angular.z = yawrate;
	pub.publish(cmd_vel);
	return;
}

void genericCallback(float pressure, double linear_x, double linear_y, double angular_x, double angular_y, int command, float battery) {
	count++;
	delay += 0.1;
	double err_pitch = -linear_x;
	double err_roll = -linear_y;
	double err_pitchrate = -angular_x;
	double err_rollrate = -angular_y;
	err_pitchsum += err_pitch;
	err_rollsum += err_roll;
	if (count == 20) {
		err_pitchsum = 0;
		err_rollsum = 0;
		count = 1;
	}
	err_rollsum /= count;
	err_pitchsum /= count;
	float err_pressure = pressure - prev_pressure;
	err_pressure_sum += err_pressure;
	prev_pressure = pressure;

	double pitch = kpp*err_pitch + kip*err_pitchsum + kdp*err_pitchrate;
	//pitch -= 5.6;
	pitch -= 5;
	double roll = kpr*err_roll + kir*err_rollsum + kdr*err_rollrate;
	//roll += 8.6;
	roll += 6;
	double kppr = 40.8+(-1.3*(4.0-battery));
	double thrust = kppr*pressure + kipr*err_pressure_sum + kdpr*err_pressure;
	double yaw = 0;
	//double thrust = 0;
	// if (delay < 10) {
	// 	thrust = 46000;
	// }else {
	// 	thrust = 42000;
	// }
	// if (pressure == NULL || pressure == 0) {
	// 	thrust =0;
	// }
	// if (command == 0 || command == 1) {
	// 	publish(pitch, roll, 0, thrust);
	// }
	if (command == 8) {
		yaw = 200;
	}
	if (command == 2) {
		thrust += 4000;
	}
	if (command == 3) {
		thrust -= 8000;
	}
	if (command == 4) {
		roll -= 20;
	}
	if (command == 5) {
		roll += 20;
	}
	if (command == 6) {
		pitch +=25;
	}
	if (command == 7) {
		pitch -= 25;
	}
	publish(pitch, roll, yaw, thrust);
	//ROS_INFO("pitch: %f   roll: %f   thrust: %f   \n", pitch, roll, thrust);
	ROS_INFO("COMMAND: %d \n", command);

	return;
}
// 0/1:hover 2:higher 3:lower 4:roll left 5:roll right 6:pitch forward 7:pitch backwards
void gestureCallback(const geometry_msgs::Quaternion &gesture_msg) {
	double gesture_x = gesture_msg.x;
	double gesture_y = gesture_msg.y;
	double gesture_z = gesture_msg.z;
	double gesture_hold = gesture_msg.w;

	//if left hand is up, hold the drone hover no matter what right hand is doing
	if (gesture_hold >= 0.1) {
		command = 1;
		if (gesture_y > 0.6) {
			command = 8;
		}
	} else {
		if (gesture_x < 0.1) {
			command = 4;
		} else if (gesture_z < -0.3) {
			command = 6;
		} else if (gesture_z > 0.2) {
			command = 7;
		} else if (gesture_x > 0.7) {
			command = 5;
		} else if (gesture_y > 0.6) {
			command = 2;
		} else if (gesture_y < -0.2) {
			command = 3;
		}
	}

	gestureUpdate = true;
	if (batteryUpdate && pressureUpdate && imuUpdate && gestureUpdate) {
		genericCallback(pressure, linear_x, linear_y, angular_x, angular_y, command, battery);
	}
}

void pressureCallback(const std_msgs::Float32::ConstPtr &float_msg) {
	pressure = float_msg->data;
	pressureUpdate = true;
	if (batteryUpdate && pressureUpdate && imuUpdate && gestureUpdate) {
		genericCallback(pressure, linear_x, linear_y, angular_x, angular_y, command, battery);
	}
	//ROS_INFO("Pressure: %f \n", pressure);
	return;
}
void batteryCallback(const std_msgs::Float32::ConstPtr &battery_msg) {
	battery = battery_msg->data;
	batteryUpdate = true;
	if (batteryUpdate && pressureUpdate && imuUpdate && gestureUpdate) {
		genericCallback(pressure, linear_x, linear_y, angular_x, angular_y, command, battery);
	}
	return;
}
void imuCallback(const sensor_msgs::Imu &_msg) {

	angular_x = _msg.angular_velocity.x;
	angular_y = _msg.angular_velocity.y;
	linear_x = _msg.linear_acceleration.x;
	linear_y = _msg.linear_acceleration.y;
	
	//linear_x can be regarded as pitch, linear_y can be regarded as roll
	//angular_x can be regarded as pitch rate, angular_y can be regarded
	//as roll rate
	//ignore yaw at the moment since the quadcopter handles it pretty good

	imuUpdate = true;
	if (batteryUpdate && imuUpdate && pressureUpdate && gestureUpdate) {
		genericCallback(pressure, linear_x, linear_y, angular_x, angular_y, command, battery);
	}
	//publish(0, 0, 0, 38000);
	//ROS_INFO("pitch: %f      roll: %f\n", pitch, roll);
	return;
}