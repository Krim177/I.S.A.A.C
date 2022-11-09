#include <datastructures.h>



float deadband(float value, const float threshold)
{
    if (fabsf(value) < threshold)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= threshold;
    }
    else if (value < 0)
    {
        value += threshold;
    }
    return value;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


void copyQuaternion(Quaternion *p, Quaternion *q)
{
    q->w = p->w;
    q->x = p->x;
    q->y = p->y;
    q->z = p->z;
}

void normalizeQuaternion(Quaternion *q)
{
    float n = dot(q, q);
    float res = invSqrt(n);
    
    q->w *= res;
    q->x *= res;
    q->y *= res;
    q->z *= res;
}

void initQuaternion(Quaternion *q, float yaw, float pitch, float roll)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q->w = cy * cp * cr + sy * sp * sr;
    q->x = cy * cp * sr - sy * sp * cr;
    q->y = sy * cp * sr + cy * sp * cr;
    q->z = sy * cp * cr - cy * sp * sr;
   
    normalizeQuaternion(q);
}

void conjugate(Quaternion *q, Quaternion *r)
{
    r->w = -q->w;
    r->x = -q->x;
    r->y = -q->y;
    r->z =  q->z;
}

void diff(Quaternion *q, Quaternion *r, Quaternion *t)
{
    Quaternion s;
    inverse(q, &s);

    multiplyQuaternion(&s, r, t);
}

void inverse(Quaternion *q, Quaternion *r)
{
    conjugate(q, r);
    
    normalizeQuaternion(r);
}

void multiplyQuaternion(Quaternion *r, Quaternion *q, Quaternion *t)
{
    t->w = r->w * q->w - r->x * q->x - r->y * q->y - r->z * q->z;
    t->x = r->w * q->x + r->x * q->w - r->y * q->z + r->z * q->y;
    t->y = r->w * q->y + r->x * q->z + r->y * q->w - r->z * q->x;
    t->z = r->w * q->z - r->x * q->y + r->y * q->x + r->z * q->w;
    
}
 
float dot(Quaternion *q, Quaternion *r)
{
    return q->w*r->w + q->x*r->x + q->y*r->y + q->z*r->z;
}

float getAngleFromQuaternion(Quaternion *q)
{
    float siny_cosp = 2.0 * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0 - 2.0 * (q->y * q->y + q->z * q->z);  
    return atan2(siny_cosp, cosy_cosp);

}