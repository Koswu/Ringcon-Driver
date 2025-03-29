#include "joycon/joycon.hpp"

// 辅助函数：从两个字节中提取16位有符号整数
int16_t extract_int16(uint8_t lsb, uint8_t msb) {
    return uint16_to_int16(lsb | (msb << 8));
}

// 辅助函数：提取摇杆校准数据
uint16_t extract_stick_cal_value(uint8_t lsb, uint8_t msb, bool high_nibble) {
    if (high_nibble) {
        return ((msb << 8) & 0xF00) | lsb;
    } else {
        return (msb << 4) | (lsb >> 4);
    }
}

// 辅助函数：处理左侧摇杆校准数据
void process_left_stick_calibration(uint8_t* cal_data, uint16_t* stick_x, uint16_t* stick_y) {
    // 获取中心点校准值
    stick_x[1] = extract_stick_cal_value(cal_data[3], cal_data[4], true);
    stick_y[1] = extract_stick_cal_value(cal_data[4], cal_data[5], false);
    
    // 获取最小值校准值（减去偏移量）
    stick_x[0] = stick_x[1] - extract_stick_cal_value(cal_data[6], cal_data[7], true);
    stick_y[0] = stick_y[1] - extract_stick_cal_value(cal_data[7], cal_data[8], false);
    
    // 获取最大值校准值（加上偏移量）
    stick_x[2] = stick_x[1] + extract_stick_cal_value(cal_data[0], cal_data[1], true);
    stick_y[2] = stick_y[1] + extract_stick_cal_value(cal_data[2], cal_data[2], false);
}

// 辅助函数：处理右侧摇杆校准数据
void process_right_stick_calibration(uint8_t* cal_data, uint16_t* stick_x, uint16_t* stick_y) {
    // 获取中心点校准值
    stick_x[1] = extract_stick_cal_value(cal_data[9], cal_data[10], true);
    stick_y[1] = extract_stick_cal_value(cal_data[10], cal_data[11], false);
    
    // 获取最小值校准值（减去偏移量）
    stick_x[0] = stick_x[1] - extract_stick_cal_value(cal_data[12], cal_data[13], true);
    stick_y[0] = stick_y[1] - extract_stick_cal_value(cal_data[13], cal_data[14], false);
    
    // 获取最大值校准值（加上偏移量）
    stick_x[2] = stick_x[1] + extract_stick_cal_value(cal_data[15], cal_data[16], true);
    stick_y[2] = stick_y[1] + extract_stick_cal_value(cal_data[16], cal_data[17], false);
}

// 辅助函数：处理传感器校准数据
void process_sensor_calibration(uint8_t* factory_data, uint8_t* user_data, int16_t sensor_cal[2][3], float* acc_coeff, float* gyro_coeff) {
    // 加速度计工厂校准原点位置
    sensor_cal[0][0] = extract_int16(factory_data[0], factory_data[1]);
    sensor_cal[0][1] = extract_int16(factory_data[2], factory_data[3]);
    sensor_cal[0][2] = extract_int16(factory_data[4], factory_data[5]);

    // 陀螺仪工厂校准原点位置
    sensor_cal[1][0] = extract_int16(factory_data[0xC], factory_data[0xD]);
    sensor_cal[1][1] = extract_int16(factory_data[0xE], factory_data[0xF]);
    sensor_cal[1][2] = extract_int16(factory_data[0x10], factory_data[0x11]);

    // 检查是否存在用户校准数据
    if ((user_data[0x0] | user_data[0x1] << 8) == 0xA1B2) {
        // 加速度计用户校准原点位置
        sensor_cal[0][0] = extract_int16(user_data[2], user_data[3]);
        sensor_cal[0][1] = extract_int16(user_data[4], user_data[5]);
        sensor_cal[0][2] = extract_int16(user_data[6], user_data[7]);

        // 陀螺仪用户校准原点位置
        sensor_cal[1][0] = extract_int16(user_data[0xE], user_data[0xF]);
        sensor_cal[1][1] = extract_int16(user_data[0x10], user_data[0x11]);
        sensor_cal[1][2] = extract_int16(user_data[0x12], user_data[0x13]);
    }

    // 计算加速度计校准系数并转换为SI单位
    const float ACC_CONVERSION_FACTOR = 4.0f * 9.8f;
    for (int i = 0; i < 3; i++) {
        acc_coeff[i] = (1.0f / (16384.0f - sensor_cal[0][i])) * ACC_CONVERSION_FACTOR;
    }

    // 计算陀螺仪校准系数并转换为SI单位
    const float GYRO_CONVERSION_FACTOR = 0.01745329251994f; // 度到弧度的转换系数
    for (int i = 0; i < 3; i++) {
        gyro_coeff[i] = (936.0f / (13371.0f - sensor_cal[1][i])) * GYRO_CONVERSION_FACTOR;
    }
}

// 获取校准数据的主函数
void GetCalibrationData() {
    printf("Getting calibration data...\n");
    
    // 初始化所有校准数据缓冲区
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

    // 从SPI读取校准数据
    get_spi_data(0x6020, 0x18, factory_sensor_cal);
    get_spi_data(0x603D, 0x12, factory_stick_cal);
    get_spi_data(0x6080, 0x6, sensor_model);
    get_spi_data(0x6086, 0x12, stick_model);
    get_spi_data(0x6098, 0x12, &stick_model[0x12]);
    get_spi_data(0x8010, 0x16, user_stick_cal);
    get_spi_data(0x8026, 0x1A, user_sensor_cal);

    // 处理摇杆校准数据
    if (this->left_right == 1 || this->left_right == 3) {
        process_left_stick_calibration(factory_stick_cal, stick_cal_x_l, stick_cal_y_l);
    }

    if (this->left_right == 2 || this->left_right == 3) {
        process_right_stick_calibration(factory_stick_cal, stick_cal_x_r, stick_cal_y_r);
    }

    // 处理用户摇杆校准数据（如果存在）
    if ((user_stick_cal[0] | user_stick_cal[1] << 8) == 0xA1B2) {
        process_left_stick_calibration(user_stick_cal, stick_cal_x_l, stick_cal_y_l);
    }

    if ((user_stick_cal[0xB] | user_stick_cal[0xC] << 8) == 0xA1B2) {
        // 偏移量不同于工厂校准数据，因此单独处理
        stick_cal_x_r[1] = extract_stick_cal_value(user_stick_cal[13], user_stick_cal[14], true);
        stick_cal_y_r[1] = extract_stick_cal_value(user_stick_cal[14], user_stick_cal[15], false);
        stick_cal_x_r[0] = stick_cal_x_r[1] - extract_stick_cal_value(user_stick_cal[16], user_stick_cal[17], true);
        stick_cal_y_r[0] = stick_cal_y_r[1] - extract_stick_cal_value(user_stick_cal[17], user_stick_cal[18], false);
        stick_cal_x_r[2] = stick_cal_x_r[1] + extract_stick_cal_value(user_stick_cal[19], user_stick_cal[20], true);
        stick_cal_y_r[2] = stick_cal_y_r[1] + extract_stick_cal_value(user_stick_cal[20], user_stick_cal[21], false);
    }

    // 处理传感器校准数据
    process_sensor_calibration(factory_sensor_cal, user_sensor_cal, sensor_cal, acc_cal_coeff, gyro_cal_coeff);
}

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