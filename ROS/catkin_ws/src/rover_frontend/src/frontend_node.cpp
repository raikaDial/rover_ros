#include "frontend.h"

Frontend::Frontend(ros::NodeHandle nh) : m_nh(nh), m_drive_cmd_tx_interval(0.1) {
	// Setup Subscribers
	m_joy_sub = m_nh.subscribe<sensor_msgs::Joy>("joy", 10, &Frontend::joyCB, this);
	
	// Setup Publishers
	m_robot_drive_pub = m_nh.advertise<std_msgs::UInt8MultiArray>("robot_drive_power", 1);
	
	// Initialize control variable
	m_time_last_drive_cmd_txd = ros::Time::now();
	
	m_robot_drive_msg.data.clear();
	m_robot_drive_msg.data.push_back(0);
	m_robot_drive_msg.data.push_back(90);
}

void Frontend::joyCB(const sensor_msgs::Joy::ConstPtr & joy) {
	int8_t drive_power = 127*(joy -> axes[1]);
	uint8_t steering_servo_angle = 90 + 90*(joy -> axes[0]);
	
	m_robot_drive_msg.data.clear();
	m_robot_drive_msg.data.push_back((uint8_t) drive_power);
	m_robot_drive_msg.data.push_back(steering_servo_angle);
}

void Frontend::update() {
	// Publish drive commands regularly
	if((ros::Time::now() - m_time_last_drive_cmd_txd).toSec() > m_drive_cmd_tx_interval) {
		m_robot_drive_pub.publish(m_robot_drive_msg);
		m_time_last_drive_cmd_txd = ros::Time::now();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "frontend_node");
	ros::NodeHandle nh;
	
	Frontend frontend(nh);
	
	while(ros::ok()) {
		ros::spinOnce();
		frontend.update();
	}
	return 0;
}	
