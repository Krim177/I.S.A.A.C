#include <math.h>
#include "arm_math.h"


using namespace std;

#define DT	     0.5
#define VARIANCE 0.15 //Decreasing the value of variance will have less variation in the output
#define HDT      0.5*DT*DT

/*const float32_t F_MPU3[9] = {
		    HDT, 0.0, 0.0,
			0.0, HDT, 0.0,
			0.0, 0.0, HDT
};*/
const float32_t F_MPU3[9] = {
			HDT, 0.0, DT,
			0.0, HDT, DT,
			0.0, 0.0, HDT
};

const float32_t Q_MPU3[9] = {
			VARIANCE*DT*DT, VARIANCE*DT*DT, VARIANCE*DT,
			VARIANCE*DT*DT, VARIANCE*DT*DT, VARIANCE*DT,
			VARIANCE*DT,    VARIANCE*DT,    VARIANCE
};

const float32_t H_MPU3[9] = {
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0
};

const float32_t I_MPU3[9] = {
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0
};

float32_t X_MPU3[3] = {
			0.0,
			0.0,
			0.0
};

float32_t P_MPU3[9] = {
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
};

float32_t K_MPU3[9] = {
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
};

//float32_t Rx = 0.0;
float R_x =0.01, R_y = 0.01, R_z =0.01;
float32_t R_MPU3[9] = {
			R_x, 0.0, 0.0,
			0.0, R_y, 0.0,
			0.0, 0.0, R_z
};

float32_t Y_MPU3[3] = {
			0.0,
			0.0,
			0.0
};

float32_t Z_MPU3[3] = {
			0.0,
			0.0,
			0.0
};

float32_t X3x1_MPU[3] = {  0.0,
						   0.0,
						   0.0
};

float32_t X3x3_MPU[9] = {  0.0, 0.0, 0.0,
						   0.0, 0.0, 0.0,
						   0.0, 0.0, 0.0
};

float32_t X2x2_MPU[4] = {  0.0, 0.0,
						   0.0, 0.0
};

float32_t X2x1_MPU[2] = {  0.0,
						   0.0
};

float32_t X1x1_MPU[1] = {  0.0
};

arm_matrix_instance_f32 F_MPU3_arm;
arm_matrix_instance_f32 Q_MPU3_arm;
arm_matrix_instance_f32 K_MPU3_arm;
arm_matrix_instance_f32 H_MPU3_arm;
arm_matrix_instance_f32 I_MPU3_arm;
arm_matrix_instance_f32 X_MPU3_arm;
arm_matrix_instance_f32 P_MPU3_arm;
arm_matrix_instance_f32 Y_MPU3_arm;
arm_matrix_instance_f32 R_MPU3_arm;
arm_matrix_instance_f32 Z_MPU3_arm;

arm_matrix_instance_f32 X3x1_MPU_arm;
arm_matrix_instance_f32 X3x3_MPU_arm;
arm_matrix_instance_f32 X2x2_MPU_arm;
arm_matrix_instance_f32 X2x1_MPU_arm;
arm_matrix_instance_f32 X1x1_MPU_arm;

void initConstantMatrix_MPU3()
{
	arm_mat_init_f32(&F_MPU3_arm, 3, 3, (float32_t*)F_MPU3);
	arm_mat_init_f32(&Q_MPU3_arm, 3, 3, (float32_t*)Q_MPU3);
	arm_mat_init_f32(&K_MPU3_arm, 3, 3, (float32_t*)K_MPU3);
	arm_mat_init_f32(&H_MPU3_arm, 3, 3, (float32_t*)H_MPU3);
	arm_mat_init_f32(&I_MPU3_arm, 3, 3, (float32_t*)I_MPU3);
	arm_mat_init_f32(&X_MPU3_arm, 3, 1, (float32_t*)X_MPU3);
	arm_mat_init_f32(&P_MPU3_arm, 3, 3, (float32_t*)P_MPU3);
	arm_mat_init_f32(&Y_MPU3_arm, 3, 1, (float32_t*)Y_MPU3);
	arm_mat_init_f32(&R_MPU3_arm, 3, 3, (float32_t*)R_MPU3);
	arm_mat_init_f32(&Z_MPU3_arm, 3, 1, (float32_t*)Z_MPU3);
}



bool KalmanFilter_MPU3(float x_mpu, float y_mpu, float z_mpu)
{
	// P, Z, R is global

	float32_t X2[9] = {  0.0, 0.0, 0.0,
			 	 	 	 0.0, 0.0, 0.0,
						 0.0, 0.0, 0.0
	};
	float32_t X3[9] = {  0.0, 0.0, 0.0,
				 	 	 0.0, 0.0, 0.0,
						 0.0, 0.0, 0.0
	};
	float32_t X4[9] = {  0.0, 0.0, 0.0,
				 	 	 0.0, 0.0, 0.0,
						 0.0, 0.0, 0.0
	};
	float32_t X5[9] = {  0.0, 0.0, 0.0,
				 	 	 0.0, 0.0, 0.0,
						 0.0, 0.0, 0.0
	};
	float32_t X6[3] = {  0.0,
			             0.0,
						 0.0
	};
	float32_t X7[3] = {  0.0,
						 0.0,
						 0.0
	};

    Z_MPU3[0] = x_mpu;
    Z_MPU3[1] = y_mpu;
    Z_MPU3[2] = z_mpu;


	//arm_matrix_instance_f32 X1_arm;
	arm_matrix_instance_f32 X2_arm;
	arm_matrix_instance_f32 X3_arm;
	arm_matrix_instance_f32 X4_arm;
	arm_matrix_instance_f32 X5_arm;
	arm_matrix_instance_f32 X6_arm;
	arm_matrix_instance_f32 X7_arm;
	//arm_matrix_instance_f32 X8_arm;


	//arm_mat_init_f32(&X1_arm, 3, 1, (float32_t *) X1);
	arm_mat_init_f32(&X2_arm, 3, 3, (float32_t *) X2);
	arm_mat_init_f32(&X3_arm, 3, 3, (float32_t *) X3);
	arm_mat_init_f32(&X4_arm, 3, 3, (float32_t *) X4);
	arm_mat_init_f32(&X5_arm, 3, 3, (float32_t *) X5);
	arm_mat_init_f32(&X6_arm, 3, 1, (float32_t *) X6);
	arm_mat_init_f32(&X7_arm, 3, 1, (float32_t *) X7);
	//arm_mat_init_f32(&X8_arm, 2, 1, (float32_t *) X8);
	arm_mat_init_f32(&X3x1_MPU_arm, 3, 1, (float32_t *) X3x1_MPU);
	arm_mat_init_f32(&X3x3_MPU_arm, 3, 3, (float32_t *) X3x3_MPU);
	arm_mat_init_f32(&X2x2_MPU_arm, 2, 2, (float32_t *) X2x2_MPU);
	arm_mat_init_f32(&X2x1_MPU_arm, 2, 1, (float32_t *) X2x1_MPU);
	arm_mat_init_f32(&X1x1_MPU_arm, 1, 1, (float32_t *) X1x1_MPU);



	arm_mat_mult_f32( &F_MPU3_arm, &X_MPU3_arm, &X3x1_MPU_arm);
	arm_copy_f32( (float32_t *)X3x1_MPU,(float32_t *)X_MPU3, 3);
	// (3,1)  = (3,3)*(3,1)
	// XTemp = F*XPrev

	arm_mat_mult_f32( &F_MPU3_arm, &P_MPU3_arm, &X2_arm);
	arm_mat_trans_f32( &F_MPU3_arm, &X3_arm);
	arm_mat_mult_f32( &X2_arm, &X3_arm, &X3x3_MPU_arm);
	arm_mat_add_f32( &X3x3_MPU_arm, &Q_MPU3_arm, &P_MPU3_arm);
    // (3,3) = (3,3)*(3,3)*(3,3) + (3,3)
	// P = (F*P)*F' + Q


	arm_mat_mult_f32( &H_MPU3_arm, &P_MPU3_arm, &X2_arm);
	arm_mat_trans_f32( &H_MPU3_arm, &X3_arm);
	arm_mat_mult_f32( &X2_arm, &X3_arm, &X4_arm);
	arm_mat_add_f32( &X4_arm, &R_MPU3_arm, &X3x3_MPU_arm);
	arm_mat_mult_f32( &P_MPU3_arm, &X3_arm, &X2_arm);

	if (ARM_MATH_SINGULAR == arm_mat_inverse_f32( &X3x3_MPU_arm, &X5_arm))
	{
		return false;
	}

	else{
		arm_mat_inverse_f32( &X3x3_MPU_arm, &X5_arm);
		arm_mat_mult_f32( &X2_arm, &X5_arm, &K_MPU3_arm);
	}


	// (3,3) = (3,3)*(3,3)*(((3,3)*(3,3))*(3,3)) + (3,3))
	// K = (P*H')*((H*P)*H'+R)^-1

	arm_mat_mult_f32( &H_MPU3_arm, &X_MPU3_arm, &X6_arm);
	arm_mat_sub_f32( &Z_MPU3_arm, &X6_arm, &X3x1_MPU_arm);
	arm_mat_mult_f32( &K_MPU3_arm, &X3x1_MPU_arm, &X6_arm);
	arm_mat_add_f32( &X_MPU3_arm, &X6_arm, &X3x1_MPU_arm);
	arm_copy_f32((float32_t *)X3x1_MPU, (float32_t *)X_MPU3, 3);
    // (3,1) = (3,1) + (3,3)*((3,1)-((3,3)*(3,1)))
	// XPres = XTemp + k* (Z-(H*XTemp))
	// XPrev  = XPres;

	arm_mat_mult_f32( &K_MPU3_arm, &H_MPU3_arm, &X3x3_MPU_arm);
	arm_mat_sub_f32( &I_MPU3_arm, &X3x3_MPU_arm, &X2_arm);
	arm_mat_mult_f32( &P_MPU3_arm, &X2_arm, &X3x3_MPU_arm);
	arm_copy_f32 ((float32_t *)X3x3_MPU, (float32_t *)P_MPU3, 9);
	// (3,3) = ((3,3)-((3,3)*(3,3)))*(3,3)
	// P = (I-(K*H)) * P;

	arm_mat_mult_f32( &H_MPU3_arm, &X_MPU3_arm, &Y_MPU3_arm);
	// (3,1) = (3,3) * (3*1)
	// Y = H*Xpres

	// ******* Add equation for R ********//
		// 3,3)
		// R = Y- Z
		//R = MatrixSub (Y,Z);
	arm_mat_sub_f32( &Y_MPU3_arm, &Z_MPU3_arm, &X7_arm);
	//R_MPU[0] = Y_MPU[0]-Z_MPU[0];//[i*Col+j]
	R_x = X7[0];
	R_y = X7[1];
	R_z = X7[2];
	return true;
}


void getState_mpu3(float *posx_mpu, float *posy_mpu, float *posz_mpu)
{
    *posx_mpu = Y_MPU3[0];
    *posy_mpu = Y_MPU3[1];
    *posz_mpu = Y_MPU3[2];

}
