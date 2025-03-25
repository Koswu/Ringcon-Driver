#pragma once
#include <bitset>
#include <chrono>
#include <hidapi.h>
#include "tools.hpp"
#include "joycon/utils.hpp"

#define JOYCON_VENDOR 0x057e
#define JOYCON_L_BT 0x2006
#define JOYCON_R_BT 0x2007
#define PRO_CONTROLLER 0x2009
#define JOYCON_CHARGING_GRIP 0x200e
#define L_OR_R(lr) (lr == 1 ? 'L' : (lr == 2 ? 'R' : '?'))



class Joycon {



	hid_device* handle;
	wchar_t* serial;

	std::string name;

	int deviceNumber = 0;// left(0) or right(1)
	int VigemNumber = 0;// vigem device number / device group number

	bool bluetooth = true;
	bool ringconattached = false;

	int left_right = 0;// 1: left joycon, 2: right joycon, 3: pro controller

	uint16_t buttons = 0;
	uint16_t buttons2 = 0;// for pro controller

	int ringcon = 0x0A; //Ringcon data. Packet[40]. Fully pulled = 0x00, rest = 0x0a, fully pushed = 0x14.

	struct btn_states {
		// left:
		int up = 0;
		int down = 0;
		int left = 0;
		int right = 0;
		int l = 0;
		int zl = 0;
		int minus = 0;
		int capture = 0;

		// right:
		int a = 0;
		int b = 0;
		int x = 0;
		int y = 0;
		int r = 0;
		int zr = 0;
		int plus = 0;
		int home = 0;

		// shared:
		int sl = 0;
		int sr = 0;
		int stick_button = 0;

		// pro controller:
		int stick_button2 = 0;// pro controller

	} btns;

	int8_t dstick;
	uint8_t battery;

	int global_count = 0;

	struct Stick {
		uint16_t x = 0;
		uint16_t y = 0;
		float CalX = 0;
		float CalY = 0;
	};

	Stick stick;
	Stick stick2;// Pro Controller

	struct Gyroscope {
		// relative:
		float pitch = 0;
		float yaw = 0;
		float roll = 0;

		struct Offset {
			int n = 0;

			// absolute:
			float pitch = 0;
			float yaw = 0;
			float roll = 0;
		} offset;
	} gyro;

	struct Accelerometer {
		float prevX = 0;
		float prevY = 0;
		float prevZ = 0;
		float x = 0;
		float y = 0;
		float z = 0;
	} accel;


	// calibration data:

	struct brcm_hdr {
		uint8_t cmd;
		uint8_t rumble[9];
	};

	unsigned char timestampbuffer[64] = { 0 };
	// Convert long long to byte array
	void IntToByteArray(long long value)
	{
		for (int i = 0; i < 64; i++)
		{
			timestampbuffer[i] = ((value >> (8 * i)) & 0XFF);
		}
	}


	struct brcm_cmd_01 {
		uint8_t subcmd;
		uint32_t offset;
		uint8_t size;
	};

	int timing_byte = 0x0;



	float acc_cal_coeff[3];
	float gyro_cal_coeff[3];
	float cal_x[1] = { 0.0f };
	float cal_y[1] = { 0.0f };

	bool has_user_cal_stick_l = false;
	bool has_user_cal_stick_r = false;
	bool has_user_cal_sensor = false;

	unsigned char factory_stick_cal[0x12];
	unsigned char user_stick_cal[0x16];
	unsigned char sensor_model[0x6];
	unsigned char stick_model[0x24];
	unsigned char factory_sensor_cal[0x18];
	unsigned char user_sensor_cal[0x1A];
	uint16_t factory_sensor_cal_calm[0xC];
	uint16_t user_sensor_cal_calm[0xC];
	int16_t sensor_cal[0x2][0x3];
	uint16_t stick_cal_x_l[0x3];
	uint16_t stick_cal_y_l[0x3];
	uint16_t stick_cal_x_r[0x3];
	uint16_t stick_cal_y_r[0x3];


public:


	Joycon(struct hid_device_info* dev) {

		if (dev->product_id == JOYCON_CHARGING_GRIP) {

			if (dev->interface_number == 0 || dev->interface_number == -1) {
				this->name = std::string("Joy-Con (R)");
				this->left_right = 2;// right joycon
			}
			else if (dev->interface_number == 1) {
				this->name = std::string("Joy-Con (L)");
				this->left_right = 1;// left joycon
			}
		}

		if (dev->product_id == JOYCON_L_BT) {
			this->name = std::string("Joy-Con (L)");
			this->left_right = 1;// left joycon
		}
		else if (dev->product_id == JOYCON_R_BT) {
			this->name = std::string("Joy-Con (R)");
			this->left_right = 2;// right joycon
		}
		else if (dev->product_id == PRO_CONTROLLER) {
			this->name = std::string("Pro Controller");
			this->left_right = 3;// left joycon
		}

		this->serial = _wcsdup(dev->serial_number);

		printf("Found joycon %c: %ls %s\n", L_OR_R(this->left_right), this->serial, dev->path);
		this->handle = hid_open_path(dev->path);


		if (this->handle == nullptr) {
			throw;
		}
	}

	void hid_exchange(hid_device* handle, unsigned char* buf, int len) {
		if (!handle) return;

		int res;

		res = hid_write(handle, buf, len);

		//if (res < 0) {
		//	printf("Number of bytes written was < 0!\n");
		//} else {
		//	printf("%d bytes written.\n", res);
		//}

		//// set non-blocking:
		//hid_set_nonblocking(handle, 1);

		res = hid_read(handle, buf, 0x40);

	}


	void send_command(int command, uint8_t* data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		if (!bluetooth) {
			buf[0x00] = 0x80;
			buf[0x01] = 0x92;
			buf[0x03] = 0x31;
		}

		buf[bluetooth ? 0x0 : 0x8] = command;
		if (data != nullptr && len != 0) {
			memcpy(buf + (bluetooth ? 0x1 : 0x9), data, len);
		}

		hid_exchange(this->handle, buf, len + (bluetooth ? 0x1 : 0x9));

		if (data) {
			memcpy(data, buf, 0x40);
		}
	}

	void send_subcommand(int command, int subcommand, uint8_t* data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		memcpy(buf, rumble_base, 9);

		if (global_count > 0xF) {
			global_count = 0x0;
		}

		// set neutral rumble base only if the command is vibrate (0x01)
		// if set when other commands are set, might cause the command to be misread and not executed
		//if (subcommand == 0x01) {
		//	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		//	memcpy(buf + 10, rumble_base, 9);
		//}

		buf[9] = subcommand;
		if (data && len != 0) {
			memcpy(buf + 10, data, len);
		}

		send_command(command, buf, 10 + len);

		if (data) {
			memcpy(data, buf, 0x40); //TODO
		}
	}

	void rumble(int frequency, int intensity) {

		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

		// intensity: (0, 8)
		// frequency: (0, 255)

		//	 X	AA	BB	 Y	CC	DD
		//[0 1 x40 x40 0 1 x40 x40] is neutral.



		buf[1 + 0 + intensity] = 0x1;
		buf[1 + 4 + intensity] = 0x1;

		// Set frequency to increase
		if (this->left_right == 1) {
			buf[1 + 0] = frequency;// (0, 255)
		}
		else {
			buf[1 + 4] = frequency;// (0, 255)
		}

		// set non-blocking:
		hid_set_nonblocking(this->handle, 1);

		send_command(0x10, (uint8_t*)buf, 0x9);
	}

	void rumble2(uint16_t hf, uint8_t hfa, uint8_t lf, uint16_t lfa) {
		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

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

	void rumble3(float frequency, uint8_t hfa, uint16_t lfa) {

		//Float frequency to hex conversion
		if (frequency < 0.0f) {
			frequency = 0.0f;
		}
		else if (frequency > 1252.0f) {
			frequency = 1252.0f;
		}
		uint8_t encoded_hex_freq = (uint8_t)round(log2((double)frequency / 10.0) * 32.0);


		//Convert to Joy-Con HF range. Range in big-endian: 0x0004-0x01FC with +0x0004 steps.
		uint16_t hf = (encoded_hex_freq - 0x60) * 4;
		//Convert to Joy-Con LF range. Range: 0x01-0x7F.
		uint8_t lf = encoded_hex_freq - 0x40;

		rumble2(hf, hfa, lf, lfa);
	}

	void rumble4(float real_LF, float real_HF, uint8_t hfa, uint16_t lfa);
	void rumble_freq(uint16_t hf, uint8_t hfa, uint8_t lf, uint16_t lfa);
	void setGyroOffsets();
	void set_ext_config(int a, int b, int c, int d);
	void set_vib_config(int a, int b, int c, int d);
	int init_bt();
};
