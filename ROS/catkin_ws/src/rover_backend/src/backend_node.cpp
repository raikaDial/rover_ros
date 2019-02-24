#include "backend.h"

Backend::Backend(ros::NodeHandle nh) : m_nh(nh), m_drive_cmd_timeout(0.5) {
	m_robot_drive.m_drive_power = 0;
	m_robot_drive.m_steering_servo_angle = 90;
	
	m_robot_drive_sub = m_nh.subscribe<std_msgs::UInt8MultiArray>("robot_drive_power", 1000, &Backend::robotDriveCb, this);
	
	// ***** Setup the Serial Port ***** //
	// Set serial configuration
	m_serial_port = new serial::Serial(
		"",										// Port name. Found during enumeration
		115200,									// Baud rate
		serial::Timeout::simpleTimeout(1000),
		serial::eightbits,
		serial::parity_none,
		serial::stopbits_one,
		serial::flowcontrol_none
	);
	
	connectToSerial();
}

Backend::~Backend() {
	delete m_serial_port;
}

// Attempts to connect to the Arduino over serial.
void Backend::connectToSerial() {
	while(ros::ok()) {
		try{
			// Find the Arduino Port
			do {
				m_serial_port -> setPort(find_device("Arduino"));
				if(m_serial_port -> getPort() == "") {
					ROS_INFO("Error: Could not find the Arduino. Retrying...");
				}
			} while(m_serial_port -> getPort() == "");

			// Connect to the Arduino
			ROS_INFO("Connecting to port '%s'", m_serial_port -> getPort().c_str());
			m_serial_port -> open();
			ROS_INFO("Connecting to port '%s' succeeded.", m_serial_port -> getPort().c_str());
			ros::Duration(1.0).sleep();
			break;
		}
		catch(serial::IOException & e) {
			ROS_INFO("Error Connecting to serial port. Retrying...");
		}
		catch(std::invalid_argument & e) {
			ROS_INFO("Error Connecting to serial port. Retrying...");
		}
	}	
}

void Backend::sendDriveCommand() {
	// Construct packet payload and transmit
	std::vector<uint8_t> data;
	data.push_back(CMD_ROBOT_DRIVE);
	data.push_back(m_robot_drive.m_drive_power);
	data.push_back((uint8_t)m_robot_drive.m_steering_servo_angle);
	transmitPacket(data);
}

void Backend::transmitPacket(const std::vector<uint8_t> & data) {
	std::vector<uint8_t> packet;
	
	// Construct packet
	packet.push_back(0xFA);
	packet.push_back((uint8_t)data.size());
	uint16_t checksum = 0;
	for(size_t i=0; i<data.size(); ++i) {
		packet.push_back(data[i]);
		checksum += data[i];
	}
	packet.push_back((uint8_t)(checksum >> 8));
	packet.push_back((uint8_t)(checksum & 0xFF));
	
	try {
		// Transmit Packet
		std::cout << "Sending Packet: ";
		for(int i=0; i< packet.size(); ++i) {
			printf("%u ", packet[i]);
		}
		std::cout << std::endl;
		m_serial_port -> write(packet);
	}
	catch(serial::IOException & e) {
		// We lost connection to the Arduino. Attempt to reconnect.
		connectToSerial();
	}
}

void Backend::robotDriveCb(const std_msgs::UInt8MultiArray::ConstPtr & msg) {
	
	m_robot_drive.m_drive_power = (int8_t) msg -> data[0];
	m_robot_drive.m_steering_servo_angle = msg -> data[1];
	m_time_last_drive_cmd_rxd = ros::Time::now();
}

void Backend::update() {
	// Check for lost comms with frontend and stop rover if needed.
	if((ros::Time::now() - m_time_last_drive_cmd_rxd).toSec() > m_drive_cmd_timeout) {
		m_robot_drive.m_drive_power=0;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "backend_node");
	ros::NodeHandle nh;
	
	Backend backend(nh);
	
	// Main Loop Timing Variables
	double seconds_per_transmit = 0.1;
	ros::Time time_last_transmit = ros::Time::now();
	
	while(ros::ok()) {
		if(ros::Time::now().toSec() - time_last_transmit.toSec() >= seconds_per_transmit) {
			backend.sendDriveCommand();
			time_last_transmit = ros::Time::now();
		}
		ros::spinOnce();
		backend.update();
	}
	
	return 0;
}
