#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

void initConstantMatrix();
bool KalmanFilter_Flow(float x, float y, float gx, float gy, float ax, float ay, float *XPos, float *YPos,float *XVel,float *YVel);




#endif /* KALMAN_FILTER_H_ */
