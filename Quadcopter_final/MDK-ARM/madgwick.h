#ifndef MADGWICK_H
#define MADGWICK_H

void Madgwick_Init(float sampleFreq);
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az);

extern float q0, q1, q2, q3;


#endif
