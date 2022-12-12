#include "kalman_filter.h"
#include "complementary_filter.h"
#include "math.h"

#define PI 3.141592


Kalman_t kalmanX = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt) {
	Kalman->X[0] += (gyro - Kalman->X[1]) * dt;
	
	Kalman->P[0][0] += (Kalman->P[1][1] * dt - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q[0][0]) * dt;
	Kalman->P[0][1] -= Kalman->P[1][1] * dt;
	Kalman->P[1][0] -= Kalman->P[1][1] * dt;
	Kalman->P[1][1] += Kalman->Q[1][1] * dt;
	
	float y = acc - Kalman->X[0];
	float s = Kalman->P[0][0] + Kalman->R;
	Kalman->K[0] = Kalman->P[0][0] / s;
	Kalman->K[1] = Kalman->P[1][0] / s;
	
	Kalman->X[0] += Kalman->K[0] * y;
	Kalman->X[1] += Kalman->K[1] * y;
	
	float PZZ = Kalman->P[0][0];
	float PZO = Kalman->P[0][1];
	Kalman->P[0][0] -= Kalman->K[0] * PZZ;
	Kalman->P[0][1] -= Kalman->K[0] * PZO;
	Kalman->P[1][0] -= Kalman->K[1] * PZZ;
	Kalman->P[1][1] -= Kalman->K[1] * PZO;
}

void kalman(float* raw, float* filtered, float dt) {
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];
	
	kalmanFilter(&kalmanX, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY, acc[1], gyro[1], dt);
	
	filtered[0] = kalmanX.X[0];
	filtered[1] = kalmanY.X[0];
	filtered[2] += (raw[5] * dt);
}
