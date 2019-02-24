#ifndef FRONTEND_H_INCLUDED
#define FRONTEND_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Joy.h>

class Frontend {
	public:
		Frontend(ros::NodeHandle nh);
		
		void joyCB(const sensor_msgs::Joy::ConstPtr & joy);
		
		void update();
		
	private:
		ros::NodeHandle m_nh;
		
		// Subscribers
		ros::Subscriber m_joy_sub;
		
		// Publishers
		ros::Publisher m_robot_drive_pub;
		
		// Messages
		std_msgs::UInt8MultiArray m_robot_drive_msg;
		
		ros::Time m_time_last_drive_cmd_txd;
		double m_drive_cmd_tx_interval;
};

#endif // FRONTEND_H_INCLUDED
