#ifndef KALMANFILTER_3INPUT_H_
#define KALMANFILTER_3INPUT_H_

void initConstantMatrix_MPU3();
void KalmanFilter_MPU3(float x_mpu, float y_mpu, float z_mpu);
void getState_mpu3(float *posx_mpu, float *posy_mpu, float *posz_mpu);




#endif /* KALMANFILTER_3INPUT_H_ */
