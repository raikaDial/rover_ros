#ifndef BACKEND_H_INCLUDED
#define BACKEND_H_INCLUDED

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8MultiArray.h>
#include <RobotCommands.h>
#include <string>
#include <vector>

// ***** Serial port helper function ***** //
// find_device()
// Given an identifier string, searches through an enumerated list of serial
//     ports for a matching device. Returns the first such device found, or
//     the null string if there are no matches.
static std::string find_device(std::string id) {
	// Search through available ports for desired device
	std::vector<serial::PortInfo> devices_found = serial::list_ports();
	std::vector<serial::PortInfo>::iterator iter;
	std::string port;
	for(iter = devices_found.begin(); iter != devices_found.end(); ++iter) {
		if( (iter -> description).find(id) != std::string::npos ) {
			ROS_INFO("Found device '%s' at port '%s'",
				(iter->description).c_str(), 
				(iter->port).c_str()
			);
			return (iter->port);
		}
	}
	return "";
}

class Backend {
	public:
		Backend(ros::NodeHandle nh);
		~Backend();
		
		void connectToSerial();
		void sendDriveCommand();
		void transmitPacket(const std::vector<uint8_t> & data);
		
		void robotDriveCb(const std_msgs::UInt8MultiArray::ConstPtr & msg);
		
		void update();

	private:	
		ros::NodeHandle m_nh;
		serial::Serial* m_serial_port;
		
		RobotDrive m_robot_drive;
		RobotLED m_robot_LED;
		
		ros::Time m_time_last_drive_cmd_rxd;
		// If it's been longer than this since receiving the last drive command,
		//     stop the rover.
		double m_drive_cmd_timeout;  
		
		// Subscribers
		ros::Subscriber m_robot_drive_sub;
};

#endif // BACKEND_H_INCLUDED
