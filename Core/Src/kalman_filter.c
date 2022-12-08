#include "kalman_filter.h"
#include "complementary_filter.h"
#include "math.h"

#define PI 3.141592
#define alpha 0.98

Kalman_t kalmanX = {
	.Q_angle = 0.001f,
	.Q_bias = 0.003f,
	.R_measure = 0.03f,
	.K_angle = 0.0f,
	.K_bias = 0.0f,
	.P = {
		{0.0f, 0.0f}, 
		{0.0f, 0.0f}
	}
};

Kalman_t kalmanY = {
	.Q_angle = 0.001f,
	.Q_bias = 0.003f,
	.R_measure = 0.03f,
	.K_angle = 0.0f,
	.K_bias = 0.0f,
	.P = {
		{0.0f, 0.0f}, 
		{0.0f, 0.0f}
	}
};

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt) {
	Kalman->K_angle += (gyro - Kalman->K_bias) * dt;
	
	Kalman->P[0][0] += (Kalman->P[1][1] * dt - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= Kalman->P[1][1] * dt;
	Kalman->P[1][0] -= Kalman->P[1][1] * dt;
	Kalman->P[1][1] += Kalman->Q_bias * dt;
	
	float KZZ = Kalman->P[0][0] / (Kalman->P[0][0] + Kalman->R_measure);
	float KZO = Kalman->P[1][0] / (Kalman->P[0][0] + Kalman->R_measure);
	
	Kalman->K_angle += KZZ * (acc - Kalman->K_angle);
	Kalman->K_bias += KZO * (acc - Kalman->K_angle);
	
	float PZZ = Kalman->P[0][0];
	float PZO = Kalman->P[0][1];
	
	Kalman->P[0][0] -= KZZ * PZZ;
	Kalman->P[0][1] -= KZZ * PZO;
	Kalman->P[1][0] -= KZO * PZZ;
	Kalman->P[1][1] -= KZO * PZO;
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
	
	filtered[0] = acc[0];
	filtered[1] = acc[1];
	filtered[2] += (raw[5] * dt);
}