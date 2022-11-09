#include <math.h>
#include "arm_math.h"


using namespace std;

#define DT	     0.02
#define VARIANCE 0.25
#define HDT      0.5*DT*DT

const float32_t F_MPU[4] = {
			HDT, 0.0, 
			0.0, 1.0
};

const float32_t Q_MPU[4] = {
			VARIANCE*DT*DT, VARIANCE*DT,
			VARIANCE*DT, VARIANCE
};

const float32_t H_MPU[2] = {
			1.0, 0.0
};

const float32_t I_MPU[4] = {
			1.0, 0.0,
			0.0, 1.0
};

float32_t X_MPU[2] = {
			0.0,
			0.0
};

float32_t P_MPU[4] = {
			0.0, 0.0,
			0.0, 0.0
};

float32_t K_MPU[2] = {
			0.0,
			0.0
};

//float32_t Rx = 0.0;
float32_t R_MPU[1] = {
			0.4
};

float32_t Y_MPU[1] = {
			0.0
};

float32_t Z_MPU[1] = {
			0.0
};

arm_matrix_instance_f32 F_MPU_arm;
arm_matrix_instance_f32 Q_MPU_arm;
arm_matrix_instance_f32 K_MPU_arm;
arm_matrix_instance_f32 H_MPU_arm;
arm_matrix_instance_f32 I_MPU_arm;
arm_matrix_instance_f32 X_MPU_arm;
arm_matrix_instance_f32 P_MPU_arm;
arm_matrix_instance_f32 Y_MPU_arm;
arm_matrix_instance_f32 R_MPU_arm;
arm_matrix_instance_f32 Z_MPU_arm;

void initConstantMatrix_MPU()
{
	arm_mat_init_f32(&F_MPU_arm, 2, 2, (float32_t*)F_MPU);
	arm_mat_init_f32(&Q_MPU_arm, 2, 2, (float32_t*)Q_MPU);
	arm_mat_init_f32(&K_MPU_arm, 2, 1, (float32_t*)K_MPU);
	arm_mat_init_f32(&H_MPU_arm, 1, 2, (float32_t*)H_MPU);
	arm_mat_init_f32(&I_MPU_arm, 2, 2, (float32_t*)I_MPU);
	arm_mat_init_f32(&X_MPU_arm, 2, 1, (float32_t*)X_MPU);
	arm_mat_init_f32(&P_MPU_arm, 2, 2, (float32_t*)P_MPU);
	arm_mat_init_f32(&Y_MPU_arm, 1, 1, (float32_t*)Y_MPU);
	arm_mat_init_f32(&R_MPU_arm, 1, 1, (float32_t*)R_MPU);
	arm_mat_init_f32(&Z_MPU_arm, 1, 1, (float32_t*)Z_MPU);
}



void KalmanFilter_MPU(float x_mpu)
{
	// P, Z, R is global

	float32_t X2[4] = {  0.0, 0.0,
			 	 	 	 0.0, 0.0
	};
	float32_t X3[4] = {  0.0, 0.0,
				 	 	 0.0, 0.0
	};
	float32_t X4[2] = { 0.0, 0.0
	};
	float32_t X5[2] = {  0.0,
					 	 0.0
	};
	float32_t X6[1] = {  0.0
	};
	float32_t X7[1] = {  0.0
	};
	float32_t X8[2] = {  0.0,
						 0.0
	};
    Z_MPU[0] = x_mpu;


	//arm_matrix_instance_f32 X1_arm;
	arm_matrix_instance_f32 X2_arm;
	arm_matrix_instance_f32 X3_arm;
	arm_matrix_instance_f32 X4_arm;
	arm_matrix_instance_f32 X5_arm;
	arm_matrix_instance_f32 X6_arm;
	arm_matrix_instance_f32 X7_arm;
	arm_matrix_instance_f32 X8_arm;


	//arm_mat_init_f32(&X1_arm, 3, 1, (float32_t *) X1);
	arm_mat_init_f32(&X2_arm, 2, 2, (float32_t *) X2);
	arm_mat_init_f32(&X3_arm, 2, 2, (float32_t *) X3);
	arm_mat_init_f32(&X4_arm, 1, 2, (float32_t *) X4);
	arm_mat_init_f32(&X5_arm, 2, 1, (float32_t *) X5);
	arm_mat_init_f32(&X6_arm, 1, 1, (float32_t *) X6);
	arm_mat_init_f32(&X7_arm, 1, 1, (float32_t *) X7);
	arm_mat_init_f32(&X8_arm, 2, 1, (float32_t *) X8);



	arm_mat_mult_f32( &F_MPU_arm, &X_MPU_arm, &X_MPU_arm);
	// (2,1)  = (2,2)*(2,1)
	// XTemp = F*XPrev

	arm_mat_mult_f32( &F_MPU_arm, &P_MPU_arm, &X2_arm);
	arm_mat_trans_f32( &F_MPU_arm, &X3_arm);
	arm_mat_mult_f32( &X2_arm, &X3_arm, &X2_arm);
	arm_mat_add_f32( &X2_arm, &Q_MPU_arm, &P_MPU_arm);
     // (2,2) = (2,2)*(2,2)*(2,2) + (2,2)
	// P = (F*P)*F' + Q

	arm_mat_mult_f32( &H_MPU_arm, &P_MPU_arm, &X4_arm);
	arm_mat_trans_f32( &H_MPU_arm, &X5_arm);
	arm_mat_mult_f32( &X4_arm, &X5_arm, &X6_arm);
	arm_mat_add_f32( &X6_arm, &R_MPU_arm, &X6_arm);
	arm_mat_inverse_f32( &X6_arm, &X7_arm);
	arm_mat_mult_f32( &P_MPU_arm, &X5_arm, &X8_arm);
	arm_mat_mult_f32( &X8_arm, &X7_arm, &K_MPU_arm);
	// (2,1) = (2,2)*(2,1)*(((1,2)*(2,2))*(2,1)) + (1,1)
	// K = (P*H')*((H*P)*H'+R)^-1

	arm_mat_mult_f32( &H_MPU_arm, &X_MPU_arm, &X6_arm);
	arm_mat_sub_f32( &Z_MPU_arm, &X6_arm, &X6_arm);
	arm_mat_mult_f32( &K_MPU_arm, &X6_arm, &X5_arm);
	arm_mat_add_f32( &X_MPU_arm, &X5_arm, &X_MPU_arm);
    //(2,1) = (2,1) + (2,1)*((1,1)-((1,2)*(2,1)))
	// XPres = XTemp + k* (Z-(H*XTemp))
	//XPrev  = XPres;

	arm_mat_mult_f32( &K_MPU_arm, &H_MPU_arm, &X2_arm);
	arm_mat_sub_f32( &I_MPU_arm, &X2_arm, &X2_arm);
	arm_mat_mult_f32( &P_MPU_arm, &X2_arm, &P_MPU_arm);
	// (2,2) = ((2,2)-((2,1)*(1,2)))*(2,2)
	// P = (I-(K*H)) * P;

	arm_mat_mult_f32( &H_MPU_arm, &X_MPU_arm, &Y_MPU_arm);
	// (1,1) = (1,2) * (2*1)
	// Y = H*Xpres

	// ******* Add equation for R ********//
		// (1,1)
		// R = Y- Z
		//R = MatrixSub (Y,Z);
	//arm_mat_sub_f32( &Y_MPU_arm, &Z_MPU_arm, &R_MPU_arm);
	//R_MPU[0] = Y_MPU[0]-Z_MPU[0];//[i*Col+j]

}


void getState_mpu(float *posx_mpu)
{
    *posx_mpu= Y_MPU[0];

}


