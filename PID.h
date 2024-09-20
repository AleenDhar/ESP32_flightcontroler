#ifndef PID_H
#define PID_H

#include <sys/time.h>
#include <algorithm>

class PID
{
public:
    PID(float p, float i, float d);
    void setConstants(float p, float i, float d);
    float compute(float input, float setpoint);

private:
    float kp, ki, kd, lastError, integral;
    long  lastTime;
    struct timeval tv;
};

#endif // PID_H
