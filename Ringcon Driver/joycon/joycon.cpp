#include "joycon/joycon.hpp"

void Joycon::rumble4(float real_LF, float real_HF, uint8_t hfa, uint16_t lfa) {

	real_LF = clamp(real_LF, 40.875885f, 626.286133f);
	real_HF = clamp(real_HF, 81.75177, 1252.572266f);


	uint16_t hf = ((uint8_t)round(log2((double)real_HF * 0.01) * 32.0) - 0x60) * 4;
	uint8_t lf = (uint8_t)round(log2((double)real_LF * 0.01) * 32.0) - 0x40;

	rumble2(hf, hfa, lf, lfa);
}

void Joycon::rumble_freq(uint16_t hf, uint8_t hfa, uint8_t lf, uint16_t lfa) {
	unsigned char buf[0x400];
	memset(buf, 0, 0x40);


	//int hf		= HF;
	//int hf_amp	= HFA;
	//int lf		= LF;
	//int lf_amp	= LFA;
	// maybe:
	//int hf_band = hf + hf_amp;

	int off = 0;// offset
	if (this->left_right == 2) {
		off = 4;
	}


	// Byte swapping
	buf[0 + off] = hf & 0xFF;
	buf[1 + off] = hfa + ((hf >> 8) & 0xFF); //Add amp + 1st byte of frequency to amplitude byte

											 // Byte swapping
	buf[2 + off] = lf + ((lfa >> 8) & 0xFF); //Add freq + 1st byte of LF amplitude to the frequency byte
	buf[3 + off] = lfa & 0xFF;


	// set non-blocking:
	hid_set_nonblocking(this->handle, 1);

	send_command(0x10, (uint8_t*)buf, 0x9);
}

void Joycon::setGyroOffsets() {
	float thresh = 0.1;
	if (abs(this->gyro.roll) > thresh || abs(this->gyro.pitch) > thresh || abs(this->gyro.yaw) > thresh) {
		return;
	}

	//average = current + ((newData - current) / n);
	this->gyro.offset.n += 1;
	this->gyro.offset.roll = this->gyro.offset.roll + ((this->gyro.roll - this->gyro.offset.roll) / this->gyro.offset.n);
	this->gyro.offset.pitch = this->gyro.offset.pitch + ((this->gyro.pitch - this->gyro.offset.pitch) / this->gyro.offset.n);
	this->gyro.offset.yaw = this->gyro.offset.yaw + ((this->gyro.yaw - this->gyro.offset.yaw) / this->gyro.offset.n);
	//this->gyro.offset.roll	= this->gyro.roll;
	//this->gyro.offset.pitch = this->gyro.pitch;
	//this->gyro.offset.yaw	= this->gyro.yaw;
};



void Joycon::set_ext_config(int a, int b, int c, int d) {
	unsigned char buf[0x400];
	memset(buf, 0, 0x40);

	while (1) {
		printf("Set Ext Config 58\n");

		static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
		int res = 0;

		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		hdr->cmd = 0x01;
		hdr->rumble[0] = timing_byte & 0xF;
		timing_byte++;
		buf[10] = 0x58;
		buf[11] = a;
		buf[12] = b;
		buf[13] = c;
		buf[14] = d;

		/*for (int i = 0; i <= 48; i++) {
			printf("%i: %02x ", i, buf[i]);
		}
		printf("\n");
		printf("\n");*/

		res = hid_write(handle, buf, output_buffer_length);
		int retries = 0;

		/*for (int i = 0; i < 50; i++) {
			res = hid_read_timeout(handle, buf, sizeof(buf), 64);
			for (int i = 0; i <= 100; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/
		

		//while (1) {
		//	buf[0] != 21;
		//}

		//while (1) {}

		while (1) {
			res = hid_read_timeout(handle, buf, sizeof(buf), 64);
			//for (int i = 0; i <= 60; i++) {
			//	printf("%i: %02x ", i, buf[i]);
			//}
			//printf("\n");
			//printf("\n");
			if (buf[0] == 0x21) {
				if (buf[14] == 0x58) {
					return;
				}
			}
			retries++;
			if (retries > 8 || res == 0) {
				break;
			}
		}
	}
}


void Joycon::set_vib_config(int a, int b, int c, int d) {
	unsigned char buf[0x400];
	memset(buf, 0, 0x40);

	static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
	int res = 0;

	memset(buf, 0, sizeof(buf));
	auto hdr = (brcm_hdr*)buf;
	hdr->cmd = 0x10;
	hdr->rumble[0] = timing_byte & 0xF;
	timing_byte++;
	buf[6] = a;
	buf[7] = b;
	buf[8] = c;
	buf[9] = d;

	/*for (int i = 0; i <= 48; i++) {
		printf("%i: %02x ", i, buf[i]);
	}
	printf("\n");
	printf("\n");*/

	res = hid_write(handle, buf, output_buffer_length);

	res = hid_read_timeout(handle, buf, sizeof(buf), 64);
	for (int i = 0; i <= 100; i++) {
		printf("%i: %02x ", i, buf[i]);
	}
}


void Joycon::send_subcommand(int command, int subcommand, uint8_t* data, int len) {
	unsigned char buf[0x40];
	memset(buf, 0, 0x40);

	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
	memcpy(buf, rumble_base, 9);

	if (global_count > 0xF) {
		global_count = 0x0;
	}

	buf[9] = subcommand;
	if (data && len != 0) {
		memcpy(buf + 10, data, len);
	}

	send_command(command, buf, 10 + len);

	if (data) {
		memcpy(data, buf, 0x40); //TODO
	}
}