#ifndef XUAN_FW_FREERTOSTASKS_H
#define XUAN_FW_FREERTOSTASKS_H

#include <Arduino.h>
#include "Robot.h"

extern TaskHandle_t handleTaskRobotControl;
extern TaskHandle_t handleTaskPrint;
extern TaskHandle_t handleTaskServoLerp;

void InitTasks();

[[noreturn]] void TaskRobotControl(void *parameter);

[[noreturn]] void TaskPrint(void *parameter);

[[noreturn]] void TaskServoLerp(void *parameter);

#endif //XUAN_FW_FREERTOSTASKS_H
