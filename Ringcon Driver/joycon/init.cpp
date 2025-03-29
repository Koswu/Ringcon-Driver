#include "joycon/joycon.hpp"

// 基本初始化设置
int init_basic_settings() {
    unsigned char buf[0x40];
    memset(buf, 0, 0x40);

    // set blocking to ensure command is received:
    hid_set_nonblocking(this->handle, 0);

    // Enable vibration
    printf("Enabling vibration...\n");
    buf[0] = 0x01; // Enabled
    send_subcommand(0x1, 0x48, buf, 1);

    // Enable IMU data
    printf("Enabling IMU data...\n");
    buf[0] = 0x01; // Enabled
    send_subcommand(0x01, 0x40, buf, 1);

    // Set input report mode (to push at 60hz)
    printf("Set input report mode to 0x30...\n");
    buf[0] = 0x30;
    send_subcommand(0x01, 0x03, buf, 1);

    GetCalibrationData();
    
    return 0;
}

// 启用MCU数据
int enable_mcu_data() {
    unsigned char buf[0x40];
    memset(buf, 0, 0x40);
    int retries2 = 0;
    
    while (1) {
        printf("Enabling MCU data 22 1\n");
        buf[0] = 0x01; // Enabled - 00 = suspend, 01 = resume
        send_subcommand(0x01, 0x22, buf, 1);

        int retries = 0;
        while (1) {
            int res = hid_read_timeout(handle, buf, sizeof(buf), 64);
            if (*(u16*)&buf[0xD] == 0x2280)
                return 0;

            retries++;
            if (retries > 8 || res == 0)
                break;
        }
        retries2++;
    }
    
    return -1;
}

// 设置MCU模式为Ringcon
int set_mcu_mode_ringcon() {
    unsigned char buf[0x40];
    int retries2 = 0;
    
    // 设置MCU模式
    while (1) {
        printf("Enabling MCU data 21 21 0 3...\n");
        static int output_buffer_length = 49;
        int res = 0;

        memset(buf, 0, sizeof(buf));
        auto hdr = (brcm_hdr*)buf;
        hdr->cmd = 0x01;
        hdr->rumble[0] = timing_byte & 0xF;
        timing_byte++;
        buf[10] = 0x21;
        buf[11] = 0x21;
        buf[12] = 0x00;
        buf[13] = 0x03;

        buf[48] = mcu_crc8_calc(buf + 12, 36);

        res = hid_write(handle, buf, output_buffer_length);
        int retries = 0;

        while (1) {
            res = hid_read_timeout(handle, buf, sizeof(buf), 64);
            if (buf[0] == 0x21) {
                if (buf[15] == 0x01 && buf[22] == 0x03) // Mcu mode is standby
                    return 0;
            }
            retries++;
            if (retries > 8 || res == 0) {
                break;
            }
        }
        retries2++;
    }
    
    return -1;
}

// 设置外部设备模式
int set_external_device_mode() {
    unsigned char buf[0x40];
    
    while (1) {
        printf("Enabling MCU data 21 21 1 1...\n");

        static int output_buffer_length = 49;
        int res = 0;

        memset(buf, 0, sizeof(buf));
        auto hdr = (brcm_hdr*)buf;
        hdr->cmd = 0x01;
        hdr->rumble[0] = timing_byte & 0xF;
        timing_byte++;
        buf[10] = 0x21;
        buf[11] = 0x21;
        buf[12] = 0x01;
        buf[13] = 0x01;

        buf[48] = mcu_crc8_calc(buf + 12, 36);

        res = hid_write(handle, buf, output_buffer_length);
        int retries = 0;

        while (1) {
            res = hid_read_timeout(handle, buf, sizeof(buf), 64);
            if (buf[0] == 0x21) {
                if (buf[15] == 0x09 && buf[17] == 0x01) // Mcu mode is external ready
                    return 0;
            }
            retries++;
            if (retries > 8 || res == 0)
                break;
        }
    }
    
    return -1;
}

// 检测Ringcon连接状态
int detect_ringcon() {
    unsigned char buf[0x40];
    int retries2 = 0;
    
    printf("Get ext data 59.");
    while (1) {
        printf(".");

        static int output_buffer_length = 49;
        int res = 0;

        memset(buf, 0, sizeof(buf));
        auto hdr = (brcm_hdr*)buf;
        hdr->cmd = 0x01;
        hdr->rumble[0] = timing_byte & 0xF;
        timing_byte++;
        buf[10] = 0x59;

        res = hid_write(handle, buf, output_buffer_length);
        int retries = 0;

        while (1) {
            res = hid_read_timeout(handle, buf, sizeof(buf), 64);

            if (buf[0] == 0x21) {
                if (buf[14] == 0x59 && buf[16] == 0x20) {
                    return 1; // Ringcon已连接
                }
            }
            retries++;
            if (retries > 8 || res == 0)
                break;
        }
        retries2++;
        if (retries2 > 28 || res == 0) {
            return 0; // 未检测到Ringcon
        }
    }
    
    return -1;
}

// 配置Ringcon IMU
int configure_ringcon_imu(int Ringconretries) {
    unsigned char buf[0x40];
    
    ringconattached = true;
    printf("Enabling IMU data 3...\n");
    buf[0] = 0x03; // Ringcon IMU enabled 
    send_subcommand(0x01, 0x40, buf, 1);
    int res = 0;
    int retries = 0;
    while (1) {
        res = hid_read_timeout(handle, buf, sizeof(buf), 64);
        if (buf[0] == 0x21) {
            if (buf[14] == 0x40) {
                printf("Enabling IMU data 1...\n");
                buf[0] = 0x02;
                send_subcommand(0x01, 0x40, buf, 1);
                buf[0] = 0x01;  
                send_subcommand(0x01, 0x40, buf, 1);
                return Ringconretries + 1;
            }
        }
        retries++;
        if (retries > 20 || res == 0)
            break;
    }
    
    return -1;
}

// 配置扩展设备格式
int configure_ext_format() {
    unsigned char buf[0x40];
    
    while (1) {
        printf("Get ext dev in format config 5C...\n");

        static int output_buffer_length = 49;
        int res = 0;

        memset(buf, 0, sizeof(buf));
        auto hdr = (brcm_hdr*)buf;
        hdr->cmd = 0x01;
        hdr->rumble[0] = timing_byte & 0xF;
        timing_byte++;
        buf[10] = 0x5C;
        buf[11] = 0x06;
        buf[12] = 0x03;
        buf[13] = 0x25;
        buf[14] = 0x06;
        buf[19] = 0x1C;
        buf[20] = 0x16;
        buf[21] = 237;
        buf[22] = 52;
        buf[23] = 54;
        buf[27] = 10;
        buf[28] = 100;
        buf[29] = 11;
        buf[30] = 230;
        buf[31] = 169;
        buf[32] = 34;
        buf[33] = 0x00;
        buf[34] = 0x00;
        buf[35] = 0x04;
        buf[43] = 0x90;
        buf[44] = 0xA8;
        buf[45] = 225;
        buf[46] = 52;
        buf[47] = 54;

        res = hid_write(handle, buf, output_buffer_length);
        int retries = 0;
        while (1) {
            res = hid_read_timeout(handle, buf, sizeof(buf), 64);
            if (buf[0] == 0x21) {
                if (buf[14] == 0x5C) { // Ringcon config set
                    return 0;
                }
            }
            retries++;
            if (retries > 8 || res == 0)
                break;
        }
    }
    
    return -1;
}

// 启动外部轮询
int start_external_polling() {
    unsigned char buf[0x40];
    
    while (1) {
        printf("Start external polling 5A...\n");

        static int output_buffer_length = 49;
        int res = 0;

        memset(buf, 0, sizeof(buf));
        auto hdr = (brcm_hdr*)buf;
        hdr->cmd = 0x01;
        hdr->rumble[0] = timing_byte & 0xF;
        timing_byte++;
        buf[10] = 0x5A;
        buf[11] = 0x04;
        buf[12] = 0x01;
        buf[13] = 0x01;
        buf[14] = 0x02;

        res = hid_write(handle, buf, output_buffer_length);
        int retries = 0;

        while (1) {
            res = hid_read_timeout(handle, buf, sizeof(buf), 64);
            if (buf[0] == 0x21) {
                if (buf[14] == 0x5A) {// Mcu mode is ringcon polling
                    return 0;
                }
            }
            retries++;
            if (retries > 8 || res == 0)
                break;
        }
    }
    
    return -1;
}

// 尝试初始化Ringcon
// 返回值: true=初始化成功或不需要Ringcon; false=初始化失败
bool Joycon::try_init_ringcon(int& retries, bool is_retry) {
    // 只有重试时才重新获取校准数据
    if (is_retry) {
        GetCalibrationData();
    }
    
    // 启用MCU数据
    if (enable_mcu_data() != 0) {
        printf("Failed to enable MCU data\n");
        return false;
    }
    
    // 设置MCU模式为Ringcon
    if (set_mcu_mode_ringcon() != 0) {
        printf("Failed to set MCU mode\n");
        return false;
    }
    
    // 设置外部设备模式
    if (set_external_device_mode() != 0) {
        printf("Failed to set external device mode\n");
        return false;
    }
    
    // 检测Ringcon是否连接
    int ringcon_detected = detect_ringcon();
    if (ringcon_detected == 0) {
        printf("Enabling IMU data...\n");
        unsigned char buf[0x40];
        buf[0] = 0x01; // Enabled
        send_subcommand(0x01, 0x40, buf, 1);
        
        printf("Successfully initialized but no Ringcon detected %s!\n", this->name.c_str());
        return true;
    }
    
    // 配置Ringcon IMU
    int new_retries = configure_ringcon_imu(retries);
    if (new_retries > 0) {
        retries = new_retries;
        // 如果需要重试并且尚未重试过
        if (retries <= 1 && !is_retry) {
            // 需要重试
            return false;
        }
    }
    
    // 配置扩展设备格式
    if (configure_ext_format() != 0) {
        printf("Failed to configure extension format\n");
        return false;
    }
    
    // 启动外部轮询
    if (start_external_polling() != 0) {
        printf("Failed to start external polling\n");
        return false;
    }
    
    // 设置扩展配置
    printf("Set Ext Config 58 - 4 4 12 2...\n");
    set_ext_config(0x04, 0x04, 0x12, 0x02);
    
    return true;
}

// 主初始化函数
int Joycon::init_bt() {
    this->bluetooth = true;
    
    // 基本设置初始化
    init_basic_settings();
    
    // 左手柄或Pro控制器不需要Ringcon初始化
    if (this->left_right == 1 || this->left_right == 3) {
        printf("Successfully initialized %s!\n", this->name.c_str());
        return 0;
    }
    
    int Ringconretries = 0;
    
    // 第一次尝试初始化
    bool success = try_init_ringcon(Ringconretries, false);
    
    // 如果第一次失败且需要重试，进行第二次尝试
    if (!success && Ringconretries <= 1) {
        success = try_init_ringcon(Ringconretries, true);
    }
    
    printf("Successfully initialized %s!\n", this->name.c_str());
    return 0;
}

void Joycon::init_usb() {

	this->bluetooth = false;

	unsigned char buf[0x400];
	memset(buf, 0, 0x400);

	// set blocking:
	// this insures we get the MAC Address
	hid_set_nonblocking(this->handle, 0);

	//Get MAC Left
	printf("Getting MAC...\n");
	memset(buf, 0x00, 0x40);
	buf[0] = 0x80;
	buf[1] = 0x01;
	hid_exchange(this->handle, buf, 0x2);

	if (buf[2] == 0x3) {
		printf("%s disconnected!\n", this->name.c_str());
	}
	else {
		printf("Found %s, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", this->name.c_str(), buf[9], buf[8], buf[7], buf[6], buf[5], buf[4]);
	}

	// set non-blocking:
	//hid_set_nonblocking(jc->handle, 1);

	// Do handshaking
	printf("Doing handshake...\n");
	memset(buf, 0x00, 0x40);
	buf[0] = 0x80;
	buf[1] = 0x02;
	hid_exchange(this->handle, buf, 0x2);

	// Switch baudrate to 3Mbit
	printf("Switching baudrate...\n");
	memset(buf, 0x00, 0x40);
	buf[0] = 0x80;
	buf[1] = 0x03;
	hid_exchange(this->handle, buf, 0x2);

	//Do handshaking again at new baudrate so the firmware pulls pin 3 low?
	printf("Doing handshake...\n");
	memset(buf, 0x00, 0x40);
	buf[0] = 0x80;
	buf[1] = 0x02;
	hid_exchange(this->handle, buf, 0x2);

	//Only talk HID from now on
	printf("Only talk HID...\n");
	memset(buf, 0x00, 0x40);
	buf[0] = 0x80;
	buf[1] = 0x04;
	hid_exchange(this->handle, buf, 0x2);

	// Enable vibration
	printf("Enabling vibration...\n");
	memset(buf, 0x00, 0x400);
	buf[0] = 0x01; // Enabled
	send_subcommand(0x1, 0x48, buf, 1);

	// Enable IMU data
	printf("Enabling IMU data...\n");
	memset(buf, 0x00, 0x400);
	buf[0] = 0x01; // Enabled
	send_subcommand(0x1, 0x40, buf, 1);

	printf("Successfully initialized %s!\n", this->name.c_str());
}

void Joycon::deinit_usb() {
	unsigned char buf[0x40];
	memset(buf, 0x00, 0x40);

	//Let the Joy-Con talk BT again    
	buf[0] = 0x80;
	buf[1] = 0x05;
	hid_exchange(this->handle, buf, 0x2);
	printf("Deinitialized %s\n", this->name.c_str());
}

int Joycon::get_spi_data(uint32_t offset, const uint16_t read_len, uint8_t* test_buf) {
	int res;
	uint8_t buf[0x100];
	while (1) {
		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		auto pkt = (brcm_cmd_01*)(hdr + 1);
		hdr->cmd = 1;
		hdr->rumble[0] = timing_byte;

		buf[1] = timing_byte;

		timing_byte++;
		if (timing_byte > 0xF) {
			timing_byte = 0x0;
		}
		pkt->subcmd = 0x10;
		pkt->offset = offset;
		pkt->size = read_len;

		for (int i = 11; i < 22; ++i) {
			buf[i] = buf[i + 3];
		}

		res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt));

		res = hid_read(handle, buf, sizeof(buf));

		if ((*(uint16_t*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset)) {
			break;
		}
	}
	if (res >= 0x14 + read_len) {
		for (int i = 0; i < read_len; i++) {
			test_buf[i] = buf[0x14 + i];
		}
	}

	return 0;
}

int Joycon::write_spi_data(uint32_t offset, const uint16_t write_len, uint8_t* test_buf) {
	int res;
	uint8_t buf[0x100];
	int error_writing = 0;
	while (1) {
		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		auto pkt = (brcm_cmd_01*)(hdr + 1);
		hdr->cmd = 1;
		hdr->rumble[0] = timing_byte;
		timing_byte++;
		if (timing_byte > 0xF) {
			timing_byte = 0x0;
		}
		pkt->subcmd = 0x11;
		pkt->offset = offset;
		pkt->size = write_len;
		for (int i = 0; i < write_len; i++) {
			buf[0x10 + i] = test_buf[i];
		}
		res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt) + write_len);

		res = hid_read(handle, buf, sizeof(buf));

		if (*(uint16_t*)&buf[0xD] == 0x1180)
			break;

		error_writing++;
		if (error_writing == 125) {
			return 1;
		}
	}

	return 0;

}

void GetCalibrationData() {
	printf("Getting calibration data...\n");
	memset(factory_stick_cal, 0, 0x12);
	memset(user_stick_cal, 0, 0x16);
	memset(sensor_model, 0, 0x6);
	memset(stick_model, 0, 0x12);
	memset(factory_sensor_cal, 0, 0x18);
	memset(user_sensor_cal, 0, 0x1A);
	memset(factory_sensor_cal_calm, 0, 0xC);
	memset(user_sensor_cal_calm, 0, 0xC);
	memset(sensor_cal, 0, sizeof(sensor_cal));
	memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
	memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
	memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
	memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));


	get_spi_data(0x6020, 0x18, factory_sensor_cal);
	get_spi_data(0x603D, 0x12, factory_stick_cal);
	get_spi_data(0x6080, 0x6, sensor_model);
	get_spi_data(0x6086, 0x12, stick_model);
	get_spi_data(0x6098, 0x12, &stick_model[0x12]);
	get_spi_data(0x8010, 0x16, user_stick_cal);
	get_spi_data(0x8026, 0x1A, user_sensor_cal);


	// get stick calibration data:

	// factory calibration:

	if (this->left_right == 1 || this->left_right == 3) {
		stick_cal_x_l[1] = (factory_stick_cal[4] << 8) & 0xF00 | factory_stick_cal[3];
		stick_cal_y_l[1] = (factory_stick_cal[5] << 4) | (factory_stick_cal[4] >> 4);
		stick_cal_x_l[0] = stick_cal_x_l[1] - ((factory_stick_cal[7] << 8) & 0xF00 | factory_stick_cal[6]);
		stick_cal_y_l[0] = stick_cal_y_l[1] - ((factory_stick_cal[8] << 4) | (factory_stick_cal[7] >> 4));
		stick_cal_x_l[2] = stick_cal_x_l[1] + ((factory_stick_cal[1] << 8) & 0xF00 | factory_stick_cal[0]);
		stick_cal_y_l[2] = stick_cal_y_l[1] + ((factory_stick_cal[2] << 4) | (factory_stick_cal[2] >> 4));

	}

	if (this->left_right == 2 || this->left_right == 3) {
		stick_cal_x_r[1] = (factory_stick_cal[10] << 8) & 0xF00 | factory_stick_cal[9];
		stick_cal_y_r[1] = (factory_stick_cal[11] << 4) | (factory_stick_cal[10] >> 4);
		stick_cal_x_r[0] = stick_cal_x_r[1] - ((factory_stick_cal[13] << 8) & 0xF00 | factory_stick_cal[12]);
		stick_cal_y_r[0] = stick_cal_y_r[1] - ((factory_stick_cal[14] << 4) | (factory_stick_cal[13] >> 4));
		stick_cal_x_r[2] = stick_cal_x_r[1] + ((factory_stick_cal[16] << 8) & 0xF00 | factory_stick_cal[15]);
		stick_cal_y_r[2] = stick_cal_y_r[1] + ((factory_stick_cal[17] << 4) | (factory_stick_cal[16] >> 4));
	}


	// if there is user calibration data:
	if ((user_stick_cal[0] | user_stick_cal[1] << 8) == 0xA1B2) {
		stick_cal_x_l[1] = (user_stick_cal[6] << 8) & 0xF00 | user_stick_cal[5];
		stick_cal_y_l[1] = (user_stick_cal[7] << 4) | (user_stick_cal[6] >> 4);
		stick_cal_x_l[0] = stick_cal_x_l[1] - ((user_stick_cal[9] << 8) & 0xF00 | user_stick_cal[8]);
		stick_cal_y_l[0] = stick_cal_y_l[1] - ((user_stick_cal[10] << 4) | (user_stick_cal[9] >> 4));
		stick_cal_x_l[2] = stick_cal_x_l[1] + ((user_stick_cal[3] << 8) & 0xF00 | user_stick_cal[2]);
		stick_cal_y_l[2] = stick_cal_y_l[1] + ((user_stick_cal[4] << 4) | (user_stick_cal[3] >> 4));
	}
	else {
	}

	if ((user_stick_cal[0xB] | user_stick_cal[0xC] << 8) == 0xA1B2) {
		stick_cal_x_r[1] = (user_stick_cal[14] << 8) & 0xF00 | user_stick_cal[13];
		stick_cal_y_r[1] = (user_stick_cal[15] << 4) | (user_stick_cal[14] >> 4);
		stick_cal_x_r[0] = stick_cal_x_r[1] - ((user_stick_cal[17] << 8) & 0xF00 | user_stick_cal[16]);
		stick_cal_y_r[0] = stick_cal_y_r[1] - ((user_stick_cal[18] << 4) | (user_stick_cal[17] >> 4));
		stick_cal_x_r[2] = stick_cal_x_r[1] + ((user_stick_cal[20] << 8) & 0xF00 | user_stick_cal[19]);
		stick_cal_y_r[2] = stick_cal_y_r[1] + ((user_stick_cal[21] << 4) | (user_stick_cal[20] >> 4));
	}
	else {
		//FormJoy::myform1->textBox_rstick_ucal->Text = L"R Stick User:\r\nNo calibration";
		//printf("no user Calibration data for right stick.\n");
	}

	// get gyro / accelerometer calibration data:

	// factory calibration:

	// Acc cal origin position
	sensor_cal[0][0] = uint16_to_int16(factory_sensor_cal[0] | factory_sensor_cal[1] << 8);
	sensor_cal[0][1] = uint16_to_int16(factory_sensor_cal[2] | factory_sensor_cal[3] << 8);
	sensor_cal[0][2] = uint16_to_int16(factory_sensor_cal[4] | factory_sensor_cal[5] << 8);

	// Gyro cal origin position
	sensor_cal[1][0] = uint16_to_int16(factory_sensor_cal[0xC] | factory_sensor_cal[0xD] << 8);
	sensor_cal[1][1] = uint16_to_int16(factory_sensor_cal[0xE] | factory_sensor_cal[0xF] << 8);
	sensor_cal[1][2] = uint16_to_int16(factory_sensor_cal[0x10] | factory_sensor_cal[0x11] << 8);

	// user calibration:
	if ((user_sensor_cal[0x0] | user_sensor_cal[0x1] << 8) == 0xA1B2) {

		// Acc cal origin position
		sensor_cal[0][0] = uint16_to_int16(user_sensor_cal[2] | user_sensor_cal[3] << 8);
		sensor_cal[0][1] = uint16_to_int16(user_sensor_cal[4] | user_sensor_cal[5] << 8);
		sensor_cal[0][2] = uint16_to_int16(user_sensor_cal[6] | user_sensor_cal[7] << 8);

		// Gyro cal origin position
		sensor_cal[1][0] = uint16_to_int16(user_sensor_cal[0xE] | user_sensor_cal[0xF] << 8);
		sensor_cal[1][1] = uint16_to_int16(user_sensor_cal[0x10] | user_sensor_cal[0x11] << 8);
		sensor_cal[1][2] = uint16_to_int16(user_sensor_cal[0x12] | user_sensor_cal[0x13] << 8);
	}
	else {
		//FormJoy::myform1->textBox_6axis_ucal->Text = L"\r\n\r\nUser:\r\nNo calibration";
	}

	// Use SPI calibration and convert them to SI acc unit
	acc_cal_coeff[0] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][0]))) * 4.0f * 9.8f;
	acc_cal_coeff[1] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][1]))) * 4.0f * 9.8f;
	acc_cal_coeff[2] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][2]))) * 4.0f * 9.8f;

	// Use SPI calibration and convert them to SI gyro unit
	gyro_cal_coeff[0] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][0])) * 0.01745329251994);
	gyro_cal_coeff[1] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][1])) * 0.01745329251994);
	gyro_cal_coeff[2] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][2])) * 0.01745329251994);
}