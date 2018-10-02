#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Servo.h>

#define SABERTOOTH_TX_PIN 5
#define SABERTOOTH_RX_PIN NOT_A_PIN
#define SABERTOOTH_ADDRESS 128
#define SABERTOOTH_DRIVE_MOTOR 1

#define STEERING_SERVO_PIN 6

#define LED_RED_PIN 9
#define LED_GREEN_PIN 10
#define LED_BLUE_PIN 11

// Robot Commands Sent over Serial
enum RobotCommand {
	CMD_ROBOT_NOOP,
	CMD_ROBOT_DRIVE,
	CMD_ROBOT_LED
};

struct RobotDrive {
	int8_t m_drive_power; // -127 full reverse, 0 stop, 127 full forward
	int8_t m_steering_servo_angle; // -90 deg for full left, +90 deg full right 
};

struct RobotLED {
	uint8_t m_red, m_blue, m_green;
};
	
class Robot {
	public:
		Robot();
		void processSerialPacket(uint8_t* packet, uint8_t size);
		void update();
	private:
		SoftwareSerial m_sabertooth_serial;
		Sabertooth m_sabertooth;
		Servo m_steering_servo;

		// Settings for Robot Subsystems
		RobotDrive m_robot_drive;
		RobotLED m_robot_led;

		RobotCommand m_current_robot_command;
};

#endif // ROBOT_H_INCLUDED