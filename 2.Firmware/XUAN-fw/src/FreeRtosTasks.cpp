#include "FreeRtosTasks.h"
#include "filter.h"

TaskHandle_t handleTaskRobotControl;
TaskHandle_t handleTaskServoLerp;
TaskHandle_t handleTaskDisplay;

bool motorEnable = false;

hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer()
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(handleTaskRobotControl, &xHigherPriorityTaskWoken);
}

void HandleInterrupt()
{
    motorEnable = !motorEnable;
}


void InitTasks()
{
    xTaskCreatePinnedToCore(
        TaskRobotControl,
        "TaskRobotControl",
        10000,
        NULL,
        3,
        &handleTaskRobotControl,
        0);

    xTaskCreatePinnedToCore(
        TaskServoLerp,
        "TaskServoLerp",
        10000,
        NULL,
        2,
        &handleTaskServoLerp,
        1);

    xTaskCreatePinnedToCore(
        TaskDisplay,
        "TaskDisplay",
        10000,
        NULL,
        1,
        &handleTaskDisplay,
        1);

    // 5ms period hardware timer, to invoke TaskRobotControl
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);
}


[[noreturn]]
void TaskRobotControl(void *parameter)
{
    int tick = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // Init CAN Module
    unsigned char *mCanBufByte;
    CAN_frame_t rx_frame, tx_frame;
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    ESP32Can.CANInit();

    // Init MPU6050
    Wire1.begin(32, 33);
    Wire1.setClock(400000);
    do
    {
        accelgyro.initialize();
        delay(100);
    } while (!accelgyro.testConnection());

    accelgyro.setXAccelOffset(-2695);
    accelgyro.setYAccelOffset(1311);
    accelgyro.setZAccelOffset(4557);
    accelgyro.setXGyroOffset(27);
    accelgyro.setYGyroOffset(-23);
    accelgyro.setZGyroOffset(6);

    // Init Filter
    biquadFilter_t gyroFilterLPF[3];
    biquadFilter_t accFilterLPF[3];
    for (int axis = 0; axis < 3; axis++)
    {
        biquadFilterInitLPF(&gyroFilterLPF[axis], 200, GYRO_LPF_CUTOFF_FREQ); //200Hz
        biquadFilterInitLPF(&accFilterLPF[axis], 200, ACCEL_LPF_CUTOFF_FREQ);
    }

    // Init PID
    PID_AngleX.setpoint = PID_AngleX.relax_point;
    PID_SpeedX.integralError = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        tick++;

        // Update IMU
        accelgyro.getMotion6(&(robot.ax), &(robot.ay), &(robot.az), &(robot.gx), &(robot.gy), &(robot.gz));

//        float mode2_pitch = -atan2(robot.ay, -robot.az) * RAD_TO_DEG;
//        float mode1_pitch = -atan2(robot.ay, -robot.ax) * RAD_TO_DEG;
//        float mode1_roll = -atan2(robot.az, -robot.ax) * RAD_TO_DEG;
//        robot.mode2_pitch =
//            FILTER_RATIO * (robot.mode2_pitch + robot.gx * 0.00003815) + (1 - FILTER_RATIO) * mode2_pitch;
//        robot.mode1_pitch =
//            FILTER_RATIO * (robot.mode1_pitch - robot.gz * 0.00003815) + (1 - FILTER_RATIO) * mode1_pitch;
//        robot.mode1_roll =
//            FILTER_RATIO * (robot.mode1_roll + robot.gy * 0.00003815) + (1 - FILTER_RATIO) * mode1_roll;

        robot.ax_g = (float) robot.ax / 16384.0f;
        robot.ay_g = (float) robot.ay / 16384.0f;
        robot.az_g = (float) robot.az / 16384.0f;
        robot.gx_dps = (float) robot.gx / 131.1f;
        robot.gy_dps = (float) robot.gy / 131.1f;
        robot.gz_dps = (float) robot.gz / 131.1f;

        robot.ax_g_fil = biquadFilterApply(&accFilterLPF[0], robot.ax_g);
        robot.ay_g_fil = biquadFilterApply(&accFilterLPF[1], robot.ay_g);
        robot.az_g_fil = biquadFilterApply(&accFilterLPF[2], robot.az_g);
        robot.gx_dps_fil = biquadFilterApply(&gyroFilterLPF[0], robot.gx_dps);
        robot.gy_dps_fil = biquadFilterApply(&gyroFilterLPF[1], robot.gy_dps);
        robot.gz_dps_fil = biquadFilterApply(&gyroFilterLPF[2], robot.gz_dps);

        float mode2_pitch = -atan2(robot.ay_g_fil, robot.az_g_fil) * RAD_TO_DEG;
        robot.mode2_pitch =
            COMP_FILTER_RATIO * (robot.mode2_pitch + robot.gx_dps_fil * 0.005) + (1 - COMP_FILTER_RATIO) * mode2_pitch;

        if (tick % 2) // every 10ms at 100Hz
        {
            float error = robot.mode2_pitch - PID_AngleX.setpoint;

            PID_AngleX.integralError += error;
            float i_term = constrain(PID_AngleX.i * PID_AngleX.integralError, -1000000, 1000000);

            PID_AngleX.output =
                PID_AngleX.p * error + i_term + PID_AngleX.d * robot.gx_dps_fil * 0.01; //0.01 for PID 10ms period
            PID_AngleX.output = constrain(PID_AngleX.output, -MAX_OUTPUT_MOTOR_SPEED, MAX_OUTPUT_MOTOR_SPEED);
            PID_AngleX.lastError = error;

            tx_frame.FIR.B.FF = CAN_frame_std;
            tx_frame.MsgID = 0x104;
            tx_frame.FIR.B.DLC = 8;
            tx_frame.data.u8[0] = 0x01;
            tx_frame.data.u8[1] = 0x02;
            tx_frame.data.u8[2] = 0x80;
            tx_frame.data.u8[3] = 0x23;

            float val = motorEnable ? PID_AngleX.output : 0;

            mCanBufByte = (unsigned char *) &val;
            for (int i = 4; i < 8; i++)
                tx_frame.data.u8[i] = *(mCanBufByte + i - 4);

            ESP32Can.CANWriteFrame(&tx_frame);

            // Print debug msgs.
            Serial.printf("%.3f,%.3f\n", PID_AngleX.setpoint, robot.mode2_pitch);
        } else // every 10ms at 100Hz
        {
        }

        if (tick == 5) // every 50ms at 20Hz
        {
            /*
            * Get encoder value
            */
            tx_frame.FIR.B.FF = CAN_frame_std;
            tx_frame.MsgID = 0x204;
            tx_frame.FIR.B.DLC = 2;
            tx_frame.data.u8[0] = 0x01;
            tx_frame.data.u8[1] = 0x02;
            ESP32Can.CANWriteFrame(&tx_frame);

            if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 1) == pdTRUE)
            {
                robot.encoder_speed[INERTIA] = *(float *) (rx_frame.data.u8 + 4);

                float error = robot.encoder_speed[INERTIA] - PID_SpeedX.setpoint;

                PID_SpeedX.integralError += error;

                float i_term = PID_SpeedX.i * PID_SpeedX.integralError;
                if (i_term > SX_INTERGRAL_MAX_ANGLE)
                {
                    i_term = SX_INTERGRAL_MAX_ANGLE;
                    PID_SpeedX.integralError = SX_INTERGRAL_MAX_ANGLE / PID_SpeedX.i;
                } else if (i_term < -SX_INTERGRAL_MAX_ANGLE)
                {
                    i_term = -SX_INTERGRAL_MAX_ANGLE;
                    PID_SpeedX.integralError = -SX_INTERGRAL_MAX_ANGLE / PID_SpeedX.i;
                }

                PID_SpeedX.output = PID_SpeedX.p * error + i_term;
                PID_SpeedX.output = constrain(PID_SpeedX.output, -10, 10);

                PID_AngleX.setpoint = PID_AngleX.relax_point - PID_SpeedX.output;
            }
        } else if (tick == 10) // every 50ms at 20Hz
        {
            tick = 0;

            /*
            * Get encoder value
            */
//            tx_frame.FIR.B.FF = CAN_frame_std;
//            tx_frame.MsgID = 0x204;
//            tx_frame.FIR.B.DLC = 2;
//            tx_frame.data.u8[0] = 0x00;
//            tx_frame.data.u8[1] = 0x02;
//            ESP32Can.CANWriteFrame(&tx_frame);
//
//            if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 1) == pdTRUE)
//            {
//                robot.encoder_speed[FRICTION] = *(float *) (rx_frame.data.u8 + 4);
//            }
        }
    }

    Serial.println("Ending task TaskRobotControl");
    vTaskDelete(NULL);
}


[[noreturn]] void TaskServoLerp(void *parameter)
{
    static portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = pdMS_TO_TICKS(20);

    delay(1000);

    unsigned char *mCanBufByte;
    float val;
    CAN_frame_t tx_frame;

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (abs(robot.turnSetpoint - robot.turnCurrent) > 0.3f)
            robot.turnCurrent = robot.turnCurrent + 0.05 * (robot.turnSetpoint - robot.turnCurrent);

        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x104;
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = 0x03;
        tx_frame.data.u8[1] = 0x00;
        tx_frame.data.u8[2] = 0x80;
        tx_frame.data.u8[3] = 0x23;

        val = 90 - robot.turnCurrent;
        mCanBufByte = (unsigned char *) &val;
        for (int i = 4; i < 8; i++)
            tx_frame.data.u8[i] = *(mCanBufByte + i - 4);

        ESP32Can.CANWriteFrame(&tx_frame);
    }

    Serial.println("Ending task TaskServoLerp");
    vTaskDelete(NULL);
}

[[noreturn]] void TaskDisplay(void *parameter)
{
    while (true)
    {
//        u8g2.clearBuffer();
        u8g2.setCursor(0, 10);
        u8g2.printf("Setpoint: %.3f", PID_AngleX.setpoint);
        u8g2.setCursor(0, 20);
        u8g2.printf("PitchAng: %.3f", robot.mode2_pitch);
        u8g2.sendBuffer();

        delay(100);
    }

    Serial.println("Ending task TaskDisplay");
    vTaskDelete(NULL);
}