#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

typedef struct {
	float Q_angle;
	float Q_bias;
	float R_measure;
	float K_angle;
	float K_bias;
	float P[2][2];
} Kalman_t;

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt);
void kalman(float* raw, float* filtered, float dt);

#endif