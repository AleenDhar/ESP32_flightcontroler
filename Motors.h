
#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <termios.h>

#include <libusb.h>

// These are the channels I chose on my Pololu Maestro for every ESC
#define MOTOR_FL 5
#define MOTOR_FR 3
#define MOTOR_BL 4
#define MOTOR_BR 2

class Motors
{
public:
    Motors();
    ~Motors();
    unsigned short getSpeed(unsigned char channel);
    void setSpeed(unsigned char channel, unsigned short target);
    void setToZero();

private:
    libusb_context *ctx;
    libusb_device_handle *device_handle;
    unsigned short speeds[6];
};

#endif // _MOTORS_H
