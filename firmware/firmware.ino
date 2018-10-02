#include "Robot.h"

// ***** Control Variables for Parsing Serial Data ***** //
enum SerialState {SYNC, READ_LENGTH, READ_DATA, READ_CHECKSUM};
SerialState serial_state;
uint8_t serial_data_bytes_expected = 0;
uint8_t serial_data_bytes_rxd = 0;
uint8_t serial_packet[255];
uint16_t serial_checksum = 0;
uint8_t serial_checksum_cnt = 0;
// *********** //

Robot rover;

void setup() {
	Serial.begin(115200);
}

void loop() {
	rover.update();
}

// Serial interrupt to process received data
void serialEvent() {
	while(Serial.available()) {
		uint8_t byte = Serial.read();

		// Search for 0xFA sync byte
		if((serial_state == SYNC) && (byte == 0xFA)) {
			serial_state = READ_LENGTH;
		}
		// Get length of packet
		else if(serial_state == READ_LENGTH) {
			serial_data_bytes_expected = byte;
			serial_state = READ_DATA;
		}
		// Read in data
		else if(serial_state == READ_DATA) {
			serial_packet[serial_data_bytes_rxd] = byte;
			++serial_data_bytes_rxd;

			if(serial_data_bytes_expected == serial_data_bytes_rxd) {
				serial_state = READ_CHECKSUM;
			}
		}
		// Validate data with 16-bit checksum
		else if(serial_state == READ_CHECKSUM) {
			serial_checksum += byte;

			if(++serial_checksum_cnt == 2) {
				uint16_t check = 0;
				for(size_t i=0; i<serial_data_bytes_rxd; ++i) {
					check += serial_packet[i];
				}
				if(check == serial_checksum) {
					rover.processSerialPacket(serial_packet, serial_data_bytes_rxd);
				}
				serial_data_bytes_expected = 0;
				serial_data_bytes_rxd = 0;
				serial_checksum = 0;
				serial_state = SYNC;
			}
		}
	}
}