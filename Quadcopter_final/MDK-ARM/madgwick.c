#include "madgwick.h"
#include <math.h>

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float beta = 0.1f;
static float invSampleFreq;

void Madgwick_Init(float sampleFreq) {
    invSampleFreq = 1.0f / sampleFreq;
}

void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    recipNorm = sqrtf(ax*ax + ay*ay + az*az);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    s0 = 4*q0*(q2*q2 + q3*q3) - 2*q2*ax - 2*q3*ay;
    s1 = 4*q1*(q2*q2 + q3*q3) + 2*q2*ay - 2*q3*ax;
    s2 = 4*q2*q0*q0 + 2*q0*ax + 2*q1*ay;
    s3 = 4*q3*q0*q0 + 2*q0*ay - 2*q1*ax;

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm;
    s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta*s0;
    qDot2 -= beta*s1;
    qDot3 -= beta*s2;
    qDot4 -= beta*s3;

    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}
