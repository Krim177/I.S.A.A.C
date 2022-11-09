#ifndef __KF_H__
#define __KF_H__

#include "arm_math.h"


typedef struct {

    arm_matrix_instance_f32 X;  /* nx1 */ 
    arm_matrix_instance_f32 x;  /* nx1 */   
    arm_matrix_instance_f32 u;  /* px1 */   
    arm_matrix_instance_f32 B;  /* nxp */
    arm_matrix_instance_f32 P;  /* nxn */    
    arm_matrix_instance_f32 Q;  /* nxn */
    arm_matrix_instance_f32 R;  /* mxm */ 
    arm_matrix_instance_f32 K;  /* nxm */
    arm_matrix_instance_f32 F;  /* nxn */
    arm_matrix_instance_f32 H;  /* mxn */      
    arm_matrix_instance_f32 Ht; /* nxm */
    arm_matrix_instance_f32 Ft; /* nxn */
    arm_matrix_instance_f32 Pp; /* nxn */
    arm_matrix_instance_f32 I;  /* nxn */
    arm_matrix_instance_f32 z;  /* mx1 */

    /* temporary storage */
    arm_matrix_instance_f32 nx1;  /* nx1 */
    arm_matrix_instance_f32 nxn;  /* nxn */
    arm_matrix_instance_f32 nxn_1;  /* nxn */
    arm_matrix_instance_f32 nxm;  /* nxm */
    arm_matrix_instance_f32 mxn;  /* nxn */
    arm_matrix_instance_f32 mxm_1;  /* nxn */
    arm_matrix_instance_f32 mxm_2;  /* nxn */
    arm_matrix_instance_f32 mx1;  /* mx1 */
    arm_matrix_instance_f32 X1;  /* nx1 */
    arm_matrix_instance_f32 Z1;  /* mx1 */
    
} kf_t;  



void kf_init(kf_t *kf, int n, int m, int p);
void kf_update(kf_t *kf);
bool kf_step(kf_t *kf);


#endif /* __EKF_H__ */
