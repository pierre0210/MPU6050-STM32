#include "complementary_filter.h"
#include "math.h"

#define PI 3.141592
#define alpha 0.98

void complementary(float* raw, float* filtered, float dt) {
	float acc[2];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	
	float gyro[3];
	gyro[0] = filtered[0] + (raw[3] * dt);
	gyro[1] = filtered[1] + (raw[4] * dt);
	gyro[2] = filtered[2] + (raw[5] * dt);
	
	filtered[0] = (alpha * gyro[0]) + ((1-alpha) * acc[0]);
	filtered[1] = (alpha * gyro[1]) + ((1-alpha) * acc[1]);
	filtered[2] = gyro[2];
}