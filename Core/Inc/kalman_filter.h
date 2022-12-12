#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

typedef struct {
	float Q[2][2];
	float R;
	float X[2];
	float P[2][2];
	float K[2];
} Kalman_t;

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt);
void kalman(float* raw, float* filtered, float dt);

#endif