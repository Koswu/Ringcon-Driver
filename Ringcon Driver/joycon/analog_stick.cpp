#include "joycon/joycon.hpp"

// calibrated sticks:
// Credit to Hypersect (Ryan Juckett)
// http://blog.hypersect.com/interpreting-analog-sticks/
void Joycon::CalcAnalogStick() {

	if (this->left_right == 1) {
		CalcAnalogStick2(
			this->stick.CalX,
			this->stick.CalY,
			this->stick.x,
			this->stick.y,
			this->stick_cal_x_l,
			this->stick_cal_y_l);

	}
	else if (this->left_right == 2) {
		CalcAnalogStick2(
			this->stick.CalX,
			this->stick.CalY,
			this->stick.x,
			this->stick.y,
			this->stick_cal_x_r,
			this->stick_cal_y_r);

	}
	else if (this->left_right == 3) {
		CalcAnalogStick2(
			this->stick.CalX,
			this->stick.CalY,
			this->stick.x,
			this->stick.y,
			this->stick_cal_x_l,
			this->stick_cal_y_l);

		CalcAnalogStick2(
			this->stick2.CalX,
			this->stick2.CalY,
			this->stick2.x,
			this->stick2.y,
			this->stick_cal_x_r,
			this->stick_cal_y_r);
	}
}


void Joycon::CalcAnalogStick2
(
	float& pOutX,       // out: resulting stick X value
	float& pOutY,       // out: resulting stick Y value
	uint16_t x,              // in: initial stick X value
	uint16_t y,              // in: initial stick Y value
	uint16_t x_calc[3],      // calc -X, CenterX, +X
	uint16_t y_calc[3]       // calc -Y, CenterY, +Y
)
{

	float x_f, y_f;
	// Apply Joy-Con center deadzone. 0xAE translates approx to 15%. Pro controller has a 10% () deadzone
	float deadZoneCenter = 0.15f;
	// Add a small ammount of outer deadzone to avoid edge cases or machine variety.
	float deadZoneOuter = 0.10f;

	// convert to float based on calibration and valid ranges per +/-axis
	x = clamp(x, x_calc[0], x_calc[2]);
	y = clamp(y, y_calc[0], y_calc[2]);
	if (x >= x_calc[1]) {
		x_f = (float)(x - x_calc[1]) / (float)(x_calc[2] - x_calc[1]);
	}
	else {
		x_f = -((float)(x - x_calc[1]) / (float)(x_calc[0] - x_calc[1]));
	}
	if (y >= y_calc[1]) {
		y_f = (float)(y - y_calc[1]) / (float)(y_calc[2] - y_calc[1]);
	}
	else {
		y_f = -((float)(y - y_calc[1]) / (float)(y_calc[0] - y_calc[1]));
	}

	// Interpolate zone between deadzones
	float mag = sqrtf(x_f * x_f + y_f * y_f);
	if (mag > deadZoneCenter) {
		// scale such that output magnitude is in the range [0.0f, 1.0f]
		float legalRange = 1.0f - deadZoneOuter - deadZoneCenter;
		float normalizedMag = min(1.0f, (mag - deadZoneCenter) / legalRange);
		float scale = normalizedMag / mag;
		pOutX = (x_f * scale);
		pOutY = (y_f * scale);
	}
	else {
		// stick is in the inner dead zone
		pOutX = 0.0f;
		pOutY = 0.0f;
	}
}

// SPI (@CTCaer):

