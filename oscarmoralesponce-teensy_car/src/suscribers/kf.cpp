#include <FreeRTOS.h>
#include <math.h>
#include "kf.h"


void kf_init(kf_t *kf, int n, int m, int p1)
{
    float32_t *X = (float32_t *)pvPortMalloc(sizeof(float32_t)*n);
    float32_t *X1 = (float32_t *)pvPortMalloc(sizeof(float32_t)*n);
    float32_t *nx1 = (float32_t *)pvPortMalloc(sizeof(float32_t)*n);
    float32_t *nxn = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *nxn_1 = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *nxm = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *mxn = (float32_t *)pvPortMalloc(sizeof(float32_t)*m*n);
    float32_t *mxm_1 = (float32_t *)pvPortMalloc(sizeof(float32_t)*m*m);
    float32_t *mxm_2 = (float32_t *)pvPortMalloc(sizeof(float32_t)*m*m);
    float32_t *mx1  = (float32_t *)pvPortMalloc(sizeof(float32_t)*m);
    float32_t *k  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *Pp = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *HT  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *FT  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *z1  = (float32_t *)pvPortMalloc(sizeof(float32_t)*m);

    
    arm_mat_init_f32(&kf->Ft, n, n,  FT);
    arm_mat_init_f32(&kf->Ht, n, m,  HT);
    arm_mat_init_f32(&kf->Pp, n, n, Pp);
    arm_mat_init_f32(&kf->K, n, m, k);
    arm_mat_init_f32(&kf->nx1, n, 1, nx1);
    arm_mat_init_f32(&kf->nxn, n, n, nxn);   
    arm_mat_init_f32(&kf->nxn_1, n, n, nxn_1);   
    arm_mat_init_f32(&kf->nxm, n, m, nxm);
    arm_mat_init_f32(&kf->mxn, m, n, mxn);
    arm_mat_init_f32(&kf->mxm_1, m, m, mxm_1);
    arm_mat_init_f32(&kf->mxm_2, m, m, mxm_2);
    arm_mat_init_f32(&kf->mx1, m, 1, mx1);
    arm_mat_init_f32(&kf->X, n, 1, X);
    arm_mat_init_f32(&kf->X1, n, 1, X1);
    arm_mat_init_f32(&kf->Z1, m, 1, z1);
    
    arm_mat_trans_f32(&kf->F, &kf->Ft);
    arm_mat_trans_f32(&kf->H, &kf->Ht);
 
    
}

void kf_update(kf_t *kf)
{        
    /* X = F*x + Bu     (nxn)(nx1) + (nxp)(px1) = (nx1) */
    arm_mat_mult_f32(&kf->F, &kf->x, &kf->nx1);   
    arm_mat_mult_f32(&kf->B,  &kf->u, &kf->X);
    arm_mat_add_f32(&kf->X,  &kf->nx1, &kf->X); 
      
    /* Pp = F P F^T + Q   (nxn)(nxn)(nxn) + (nxn) = (nxn)   */
    arm_mat_mult_f32(&kf->F, &kf->P, &kf->nxn);
    arm_mat_mult_f32( &kf->nxn, &kf->Ft, &kf->Pp);
    arm_mat_add_f32(&kf->Q,  &kf->Pp, &kf->Pp);

}        

bool kf_step(kf_t *kf)
{   
    /* K = Pp H^T  (H Pp H^T + R)   (nxn)(nxm) ((mxn)(nxn)(nxm) + (mxm)) =  (nxp)) */
    arm_mat_mult_f32(&kf->Pp, &kf->Ht, &kf->nxm);
    arm_mat_mult_f32(&kf->H, &kf->nxm, &kf->mxm_1); 
    arm_mat_add_f32(&kf->mxm_1,  &kf->R, &kf->mxm_1);    
    arm_status 	status = arm_mat_inverse_f32( &kf->mxm_1, &kf->mxm_2);
    if (ARM_MATH_SINGULAR == status)
    {
      return false;
    }
    arm_mat_mult_f32( &kf->nxm, &kf->mxm_2, &kf->K);
   
    
    
    /* x = X + K(z - HX)  (nx1) + (nxm)((mx1) - (mxn)(nx1)) = (nx1) */
    arm_mat_mult_f32(&kf->H, &kf->X, &kf->mx1);
    arm_mat_sub_f32(&kf->z, &kf->mx1, &kf->Z1);
    arm_mat_mult_f32(&kf->K, &kf->Z1, &kf->mx1);  
    arm_mat_add_f32(&kf->X, &kf->mx1, &kf->x);
    
    
    /* P = (I - K H) Pp    ((nxn) - (nxp)(pxn))(nxn) */
    arm_mat_mult_f32(&kf->K, &kf->H, &kf->nxn);  
    arm_mat_sub_f32(&kf->I, &kf->nxn, &kf->nxn_1);
    arm_mat_mult_f32(&kf->nxn_1, &kf->Pp, &kf->P);
    
    return true;
}
