
/*
The OpMode which controls the DataLogger thread. The DataLogger an be set to logging an OpMode, a Match,
or nothing at all. NOTE: This file is not the code for the DataLogger itself, but rather the code which
allows you to set its perameters.

 */


package org.firstinspires.ftc.teamcode._Tele;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Objects.DataLogger;


@TeleOp(name="Robot_Data_Logger", group="Iterative Opmode")

public class ___Robot_Data_Logger extends OpMode {  

    private DataLogger dataLogger = null;
    private int currentParameter = 0;

    @Override
    public void init() {
        dataLogger = new DataLogger(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down && currentParameter < 1) {
            currentParameter++;
        } else if (gamepad1.dpad_up && currentParameter > 0) {
            currentParameter--;
        }


        if (currentParameter == 0) {
            telemetry.addData("Log Mode", DataLogger.getLogMode() + "   <--");
            if (gamepad1.dpad_left && DataLogger.getLogMode().ordinal() > 0) {
                DataLogger.setLogMode(DataLogger.LogModes.values()[DataLogger.getLogMode().ordinal() - 1]);
            } else if (gamepad1.dpad_right && DataLogger.getLogMode().ordinal() < DataLogger.LogModes.values().length - 1) {
                DataLogger.setLogMode(DataLogger.LogModes.values()[DataLogger.getLogMode().ordinal() + 1]);
            }
        } else {
            telemetry.addData("Log Mode", DataLogger.getLogMode());
        }


        if (currentParameter == 1) {
            telemetry.addData("Poll Hardware Every", DataLogger.getPollInterval() + "ms   <--");
            if (gamepad1.dpad_left && DataLogger.getPollInterval() - 100 >= 100) {
                DataLogger.setPollInterval(DataLogger.getPollInterval() - 100);
            } else if (gamepad1.dpad_right) {
                DataLogger.setPollInterval(DataLogger.getPollInterval() + 100);
            }
        } else {
            telemetry.addData("Poll Hardware Every", DataLogger.getPollInterval() + "ms");
        }

        telemetry.update();
        SystemClock.sleep(100);
    }

    @Override
    public void start() {
        dataLogger.startLogging(this);
    }

    @Override
    public void loop() {
        //telemetry.addData("Data", dataLogger.getPolledData());
        telemetry.update();
    }

    @Override
    public void stop() {
        dataLogger.stopLogging(this);
    }
}

