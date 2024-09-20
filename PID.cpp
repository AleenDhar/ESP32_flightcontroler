#include "PID.h"

PID::PID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
    lastError = 0;
    integral = 0;

    gettimeofday(&tv, 0);
    lastTime = tv.tv_sec * 1000 + tv.tv_usec/1000.0 + 0.5;
}

void PID::setConstants(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

float PID::compute(float input, float setpoint)
{
    gettimeofday(&tv, 0);
    long time = tv.tv_sec * 1000 + tv.tv_usec/1000.0 + 0.5;
    long deltaTime = time - lastTime;
    float error = setpoint - input;
    long deltaError = error - lastError;

    float proportional = error * kp;
    integral += error * deltaTime * ki;
    integral = std::min(integral, (float)5000);
    integral = std::max(integral, (float)-5000);
    float derivative = deltaError * kd / deltaTime;

    lastError = error;
    lastTime = time;

    return proportional + integral + derivative;
}
