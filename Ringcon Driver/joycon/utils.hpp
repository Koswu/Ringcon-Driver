#pragma once
#include "tools.hpp"

u8 mcu_crc8_calc(u8* buf, u8 size) { //CTCAER
	u8 crc8 = 0x0;

	for (int i = 0; i < size; ++i) {
		crc8 = mcu_crc8_table[(u8)(crc8 ^ buf[i])];
	}
	return crc8;
}

u8 ringmcu_crc8_calc(u8* buf, u8 size) {
	u8 crc8 = 0x0;

	for (int i = 0; i < size; ++i) {
		crc8 = ringmcu_crc8_table[(u8)(crc8 ^ buf[i])];
	}
	return crc8;
}

