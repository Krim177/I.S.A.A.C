/****************************
*Extended Kalman filter using Arm CMSIS-DSP*
Author : Nagarathna Hema Balaji

*****************************/

#include <FreeRTOS.h>
#include <math.h>
#include "ekf.h"
#include <Arduino.h>



/***************************
void arm_mat_init_f32(arm_matrix_instance_f32 * S, uint16_t nRows, uint16_t nColumns, float32_t * pData)
Parameters:	*S	points to an instance of the floating-point matrix structure.
       	    nRows	number of rows in the matrix.
            nColumns	number of columns in the matrix.
            *pData	points to the matrix data array
 
Memory allocation is done for each of the matrices!!
As calls to pvPortMalloc() are made, the FreeRTOS allocation scheme subdivides a simple array into smaller blocks. 
The array is called the FreeRTOS heap.

F matrix contains the co-efficients of the state vector and is given by,
**************
     1     0     del t    0
     
     0     1       0     del t
F=
     0     0       1      0
     
     0     0       0      1
**************

H is a Jacobian matrix, chosen in such a way that it 
gives the required 
**************
      0     0     1     0
     
      0     0     0     1
H =
      1     0     1     0
      
      0     1     0     1
**************
arm_fill_f32 - Fills a constant value into a floating-point vector.
****************************/

void ekf_init(ekf_t *ekf, int n, int m)
{
    uint16_t i;
    float32_t *Pp = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *nxn = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *FT  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    float32_t *nxm = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *mxm = (float32_t *)pvPortMalloc(sizeof(float32_t)*m*m);
    float32_t *mxm_1 = (float32_t *)pvPortMalloc(sizeof(float32_t)*m*m);
    float32_t *K  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *HT  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*m);
    float32_t *nx1  = (float32_t *)pvPortMalloc(sizeof(float32_t)*n);
    float32_t *I = (float32_t *)pvPortMalloc(sizeof(float32_t)*n*n);
    
    arm_fill_f32(0.0f, I, n*n);
    for (i=0; i<n; i++)
    {
       I[i + n*i] = 1.0f;
    }
 
    arm_mat_init_f32(&ekf->Pp, n, n, Pp);   
    arm_mat_init_f32(&ekf->nxn, n, n, nxn);   
    arm_mat_init_f32(&ekf->Ft, n, n,  FT);
    arm_mat_init_f32(&ekf->nxm, n, m, nxm);
    arm_mat_init_f32(&ekf->mxm, m, m, mxm);
    arm_mat_init_f32(&ekf->mxm_1, m, m, mxm_1);
    arm_mat_init_f32(&ekf->K, n, m, K);
    arm_mat_init_f32(&ekf->Ht, n, m,  HT);
    arm_mat_init_f32(&ekf->nx1, m, 1, nx1);
    arm_mat_init_f32(&ekf->I, n, n, I);
}

/*****************************
Update function is written to update the state error co-variance matrix.
Pp is the state error co-variance matrix, given by,
* Pp = F P F^T + Q *
* (nxn) = (nxn)(nxn)(nxn) + (nxn) *
*****************************/
void ekf_update(ekf_t *ekf)
{  
    arm_mat_trans_f32(&ekf->F, &ekf->Ft);
    arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->nxn);
    arm_mat_mult_f32( &ekf->nxn, &ekf->Ft, &ekf->Pp);
    arm_mat_add_f32(&ekf->Pp, &ekf->Q, &ekf->Pp);  
}        

/*********************
Step function is written to update the Kalman gain matrix, K.
* K = Pp H^T / (H Pp H^T + R) *
* (nxm) = (nxn)(nxm) / ((mxn)(nxn)(nxm)+(mxm)) *

Then the state matrix, x is updated as shown below:
* x = X + K(z - Hx) *
* (nx1) = (nx1) + (nxm)((nx1) - (nxn)(nx1)) 
Matrix z has the actual measurements from the sensor.

* P = (I - K H) Pp *
*((nxn) - (nxp)(pxn))(nxn) *
The state error co-variance matix is updated again with the new Kalman gain
***********************/
bool ekf_step(ekf_t *ekf)
{   
    arm_mat_trans_f32(&ekf->H, &ekf->Ht);
    arm_mat_mult_f32(&ekf->Pp, &ekf->Ht, &ekf->nxm);
    arm_mat_mult_f32(&ekf->H, &ekf->nxm, &ekf->mxm_1);
    arm_mat_add_f32(&ekf->mxm_1, &ekf->R, &ekf->mxm_1);    
 
    arm_status status = arm_mat_inverse_f32( &ekf->mxm_1, &ekf->mxm);
    if (ARM_MATH_SINGULAR == status)
    {
       return false;
    }
    arm_mat_mult_f32( &ekf->nxm, &ekf->mxm, &ekf->K);
 
    /* x = X + K(z - Hx)   */
    arm_mat_mult_f32(&ekf->K, &ekf->Y, &ekf->nx1);  
    arm_mat_add_f32(&ekf->X, &ekf->nx1, &ekf->x);
        
    /* P = (I - K H) Pp    ((nxn) - (nxp)(pxn))(nxn) */
    arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->nxn);  
    arm_mat_sub_f32(&ekf->I, &ekf->nxn, &ekf->nxn);
    arm_mat_mult_f32(&ekf->nxn, &ekf->Pp, &ekf->P);
 
     
    
    return true;
}