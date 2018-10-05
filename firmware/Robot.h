#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <Sabertooth.h>
#include <Servo.h>
#include "RobotCommands.h"

#define SABERTOOTH_ADDRESS 128
#define SABERTOOTH_DRIVE_MOTOR 1

#define STEERING_SERVO_PIN 6

#define LED_RED_PIN 9
#define LED_GREEN_PIN 10
#define LED_BLUE_PIN 11
	
class Robot {
	public:
		Robot();
		void init();
		void processSerialPacket(uint8_t* packet, uint8_t size);
		void update();
	private:
		Sabertooth m_sabertooth;
		Servo m_steering_servo;

		// Settings for Robot Subsystems
		RobotDrive m_robot_drive;
		RobotLED m_robot_led;

		RobotCommand m_current_robot_command;
};

#endif // ROBOT_H_INCLUDED
