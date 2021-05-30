#include <Arduino.h>
#include "Preferences.h"
#include "FreeRtosTasks.h"
#include "Robot.h"
#include "Control.h"
#include "SimpleCLI.h"

Preferences config;
Robot robot;

// Create CLI Object
SimpleCLI cli;
Command cmdSave;
Command cmdAx;
Command cmdAy;
Command cmdSx;
Command cmdSy;


//HardwareSerial CtrlSerial(1);
//HardwareSerial RPiSerial(2);

void HandleInterrupt();

void setup()
{
    /*
     * Init Serials
     */
    Serial.begin(115200);
    // CtrlSerial.begin(115200, SERIAL_8N1, 14, 13);
    // RPiSerial.begin(115200, SERIAL_8N1, 38, 37);
    pinMode(26, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(26), HandleInterrupt, FALLING);


    cmdSave = cli.addCmd("save");
    cmdSave.setDescription(" Save all PID params.");

    cmdAx = cli.addCmd("ax");
    cmdAx.addArg("p", 0);
    cmdAx.addArg("i", 0);
    cmdAx.addArg("d", 0);
    cmdAx.addArg("s", 0);
    cmdAx.setDescription(" Set AngleX PID params.");

    cmdAy = cli.addCmd("ay");
    cmdAy.addArg("p", 0);
    cmdAy.addArg("i", 0);
    cmdAy.addArg("d", 0);
    cmdAy.setDescription(" Set AngleX PID params.");

    cmdSx = cli.addCmd("sx");
    cmdSx.addArg("p", 0);
    cmdSx.addArg("i", 0);
    cmdSx.addArg("d", 0);
    cmdSx.setDescription(" Set AngleX PID params.");

    cmdSy = cli.addCmd("sy");
    cmdSy.addArg("p", 0);
    cmdSy.addArg("i", 0);
    cmdSy.addArg("d", 0);
    cmdSy.setDescription(" Set AngleX PID params.");




    /*
     * Init robot hardwares
     */
    robot.Init();

    /*
     * Reload NVM configs
     */
    config.begin("XUAN", false);
    config.getBytes("PID_AngleX", (void *) &PID_AngleX, sizeof(Pid_t));
    config.getBytes("PID_AngleY", (void *) &PID_AngleY, sizeof(Pid_t));
    config.getBytes("PID_SpeedX", (void *) &PID_SpeedX, sizeof(Pid_t));
    config.getBytes("PID_SpeedY", (void *) &PID_SpeedY, sizeof(Pid_t));
    config.end();

    /*
    * Init FreeRTOS Tasks
    */
    InitTasks();
}


void loop()
{
    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');

            if (input.length() > 0)
            {
                Serial.print("# ");
                Serial.println(input);

                cli.parse(input);
            }
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            int argNum = c.countArgs();

            Serial.print("> ");
            Serial.print(c.getName());
            Serial.print(' ');

            for (int i = 0; i < argNum; ++i)
            {
                Argument arg = c.getArgument(i);
                // if(arg.isSet()) {
                Serial.print(arg.toString());
                Serial.print(' ');
                // }
            }

            Serial.println();

            if (c == cmdSave)
            {
                config.begin("XUAN", false);
                config.putBytes("PID_AngleX", (void *) &PID_AngleX, sizeof(Pid_t));
                config.putBytes("PID_AngleY", (void *) &PID_AngleY, sizeof(Pid_t));
                config.putBytes("PID_SpeedX", (void *) &PID_SpeedX, sizeof(Pid_t));
                config.putBytes("PID_SpeedY", (void *) &PID_SpeedY, sizeof(Pid_t));
                config.end();

                Serial.println("All PID param saved.");
            } else if (c == cmdAx)
            {
                if (c.getArgument("p").isSet())
                    PID_AngleX.p = c.getArgument("p").getValue().toFloat();
                if (c.getArgument("i").isSet())
                    PID_AngleX.i = c.getArgument("i").getValue().toFloat();
                if (c.getArgument("d").isSet())
                    PID_AngleX.d = c.getArgument("d").getValue().toFloat();
                if (c.getArgument("s").isSet())
                    PID_AngleX.relax_point = c.getArgument("s").getValue().toFloat();

                Serial.printf("PID of AX: %.3f,%.3f,%.3f RL:%.3f\n", PID_AngleX.p, PID_AngleX.i, PID_AngleX.d,
                              PID_AngleX.relax_point);
            } else if (c == cmdAy)
            {
                if (c.getArgument("p").isSet())
                    PID_AngleY.p = c.getArgument("p").getValue().toFloat();
                if (c.getArgument("i").isSet())
                    PID_AngleY.i = c.getArgument("i").getValue().toFloat();
                if (c.getArgument("d").isSet())
                    PID_AngleY.d = c.getArgument("d").getValue().toFloat();

                Serial.printf("PID of AY: %.3f,%.3f,%.3f\n", PID_AngleY.p, PID_AngleY.i, PID_AngleY.d);
            } else if (c == cmdSx)
            {
                if (c.getArgument("p").isSet())
                    PID_SpeedX.p = c.getArgument("p").getValue().toFloat();
                if (c.getArgument("i").isSet())
                {
                    PID_SpeedX.i = c.getArgument("i").getValue().toFloat();
                    PID_SpeedX.integralError;
                }
                if (c.getArgument("d").isSet())
                    PID_SpeedX.d = c.getArgument("d").getValue().toFloat();

                Serial.printf("PID of SX: %.6f,%.6f,%.6f\n", PID_SpeedX.p, PID_SpeedX.i, PID_SpeedX.d);
            } else if (c == cmdSy)
            {
                if (c.getArgument("p").isSet())
                    PID_SpeedY.p = c.getArgument("p").getValue().toFloat();
                if (c.getArgument("i").isSet())
                    PID_SpeedY.i = c.getArgument("i").getValue().toFloat();
                if (c.getArgument("d").isSet())
                    PID_SpeedY.d = c.getArgument("d").getValue().toFloat();

                Serial.printf("PID of SY: %.3f,%.3f,%.3f\n", PID_SpeedY.p, PID_SpeedY.i, PID_SpeedY.d);
            }
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();

            Serial.print("ERROR: ");
            Serial.println(cmdError.toString());

            if (cmdError.hasCommand())
            {
                Serial.print("Did you mean \"");
                Serial.print(cmdError.getCommand().toString());
                Serial.println("\"?");
            }
        }

    }

}