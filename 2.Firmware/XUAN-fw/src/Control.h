#ifndef XUAN_FW_CONTROL_H
#define XUAN_FW_CONTROL_H



typedef struct
{
    float p, i, d;
    float setpoint, output, integralError, lastError;
    float relax_point;
} Pid_t;

extern volatile Pid_t PID_AngleX, PID_AngleY, PID_SpeedX, PID_SpeedY;


#endif //XUAN_FW_CONTROL_H
