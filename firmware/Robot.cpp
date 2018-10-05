#include "Robot.h"
#include <SoftwareSerial.h>

Robot::Robot() 
	:  m_sabertooth(SABERTOOTH_ADDRESS, Serial3)
{}

void Robot::init() {
	// For whatever reason, this junk of code can't be in the constructor.
	//     Issue with Arduino not being initialized or something.
	//delay(1000);
	Serial3.begin(38400);
	m_sabertooth.autobaud();
	m_steering_servo.attach(STEERING_SERVO_PIN);
}

void Robot::processSerialPacket(uint8_t* packet, uint8_t size) {
	if(size >= 1) {
		// Parse command
		m_current_robot_command = packet[0];

		// Simply store command and associated data here. Perform actions outside of interrupt.
		switch(m_current_robot_command) {
			case CMD_ROBOT_DRIVE:
				if(size == 3) {
					m_robot_drive.m_drive_power = packet[1];
					m_robot_drive.m_steering_servo_angle = packet[2];
				}
				break;
			case CMD_ROBOT_LED:
				if(size == 4) {
					m_robot_led.m_red = packet[1];
					m_robot_led.m_green = packet[2];
					m_robot_led.m_blue = packet[3];
				}
				break;
			default:
				m_current_robot_command = CMD_ROBOT_NOOP;
		}
	}
	// Clear packet
	for(uint8_t i=0; i<size; ++i) {
		packet[i] = 0; 
	}
}

void Robot::update() {
	switch(m_current_robot_command) {
		case CMD_ROBOT_DRIVE:
			m_sabertooth.motor(SABERTOOTH_DRIVE_MOTOR, m_robot_drive.m_drive_power);
			m_steering_servo.write(m_robot_drive.m_steering_servo_angle);
			break;
		case CMD_ROBOT_LED:
			// Set Colors
			analogWrite(LED_RED_PIN, m_robot_led.m_red);
			analogWrite(LED_GREEN_PIN, m_robot_led.m_green);
			analogWrite(LED_BLUE_PIN, m_robot_led.m_blue);
			break;
		default:
			break;
	}
	m_current_robot_command = CMD_ROBOT_NOOP;
}
