#ifndef __KF_H__
#define __KF_H__

#include "arm_math.h"


typedef struct {

    arm_matrix_instance_f32 X;  /* nx1 */
    arm_matrix_instance_f32 x;  /* nx1 */   
    arm_matrix_instance_f32 F;  /* nxn */
    arm_matrix_instance_f32 Ft; /* nxn */
    arm_matrix_instance_f32 P;  /* nxn */    
    arm_matrix_instance_f32 Q;  /* nxn */
    arm_matrix_instance_f32 R;  /* mxm */ 
    arm_matrix_instance_f32 K;  /* nxm */
    arm_matrix_instance_f32 H;  /* mxn */      
    arm_matrix_instance_f32 Ht; /* nxm */
    arm_matrix_instance_f32 Pp; /* nxn */
    arm_matrix_instance_f32 Y;  /* mx1 */

    /* temporary storage */
    arm_matrix_instance_f32 nx1;  /* nx1 */
    arm_matrix_instance_f32 nxn;  /* nxn */
    arm_matrix_instance_f32 nxm;  /* nxm */
    arm_matrix_instance_f32 mxm;  /* nxn */
    arm_matrix_instance_f32 mxm_1;  /* nxn */
    arm_matrix_instance_f32 I;  /* nxn */
    
} ekf_t;  



void ekf_init(ekf_t *ekf, int n, int m);
void ekf_update(ekf_t *ekf);
bool ekf_step(ekf_t *ekf);


#endif /* __EKF_H__ */
