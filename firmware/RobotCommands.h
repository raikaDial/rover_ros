#ifndef ROBOT_COMMANDS_H_INCLUDED
#define ROBOT_COMMANDS_H_INCLUDED

// Robot Commands Sent over Serial
enum RobotCommand {
	CMD_ROBOT_NOOP,
	CMD_ROBOT_DRIVE,
	CMD_ROBOT_LED
};

struct RobotDrive {
	int8_t m_drive_power; // -127 full reverse, 0 stop, 127 full forward
	uint8_t m_steering_servo_angle; // 0 deg for full left, 180 deg full right 
};

struct RobotLED {
	uint8_t m_red, m_blue, m_green;
};

#endif // ROBOT_COMMANDS_H_INCLUDED