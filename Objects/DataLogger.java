/*
Data Logger log robot data to an .csv file on the control hub.
You can connect to the control hub via USB and access this file on your computer.
This allows you to view information about past matches even after the matches are over.
Must manually added ‘Logged Robot Data’ to control hub in order to function.
 */

package org.firstinspires.ftc.teamcode.Objects;

import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Static.Odometry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedHashMap;


public class DataLogger extends Thread {

    private BufferedWriter bufferedWriter;
    private static String saveFilePath = Environment.getExternalStorageDirectory().getAbsolutePath() + File.separator + "Logged Robot Data";  //ADD DEFAULT PATH
    private String fileName;
    private static final SimpleDateFormat dateTimeFormat = new SimpleDateFormat("MM-dd-yyyy, HH.mm.ss");

    public enum LogModes {OFF, OPMODE, MATCH, CONTINUOUS}
    private static LogModes logMode = LogModes.OFF;

    private static boolean run = false;
    private boolean runThisThread = false;

    private static String currentOpModeName = "NONE";
    private static long pollInterval = 100;
    private long startTime;
    private HardwareDevice[] polledHardware;
    private LinkedHashMap<String, Object> polledData = new LinkedHashMap<>();


    public DataLogger(HardwareMap h) {
        populateHardwareArray(h);
        fileName = dateTimeFormat.format(new Date());
    }

    public DataLogger(HardwareMap h, String n) {
       populateHardwareArray(h);
        fileName = n;
    }

    private void populateHardwareArray(HardwareMap h) {
        polledHardware = new HardwareDevice[h.size()];
        int index = 0;
        for (HardwareDevice device : h) {
            polledHardware[index] = device;
            index++;
        }
    }


    public <C extends OpMode> void startLogging(C callingClass) {
        switch (logMode) {
            case OFF:
                runThisThread = false;
                break;
            case OPMODE:
                runThisThread = true;
                break;
            case MATCH:
                //ONLY RUNS IF THE CLASS CALLING THIS METHOD IS A CHILD CLASS OF "OpMode"
                if (callingClass.getClass().getSuperclass().equals(LinearOpMode.class)) {
                    runThisThread = true;
                }
                break;
            case CONTINUOUS:     //HAS NOT BEEN IMPLEMENTED YET
                //ONLY RUNS IF ANOTHER DATALOGGER THREAD IS NOT CURRENTLY RUNNING
                if (!run) {
                    runThisThread = true;
                }
                break;
        }

        currentOpModeName = callingClass.getClass().getSimpleName();

        //IF THE THREAD SHOULD START, THEN DO THE INITIAL LOGGER SETUP STUFF
        if (runThisThread) {
            startTime = System.currentTimeMillis();
            pollHardware();

            //CREATES FILE WRITER OBJECT AND CSV FILE AND THEN WRITES COLUMN TITLES TO THE FILE
            try {
                bufferedWriter = new BufferedWriter(new FileWriter(new File(saveFilePath + File.separator + fileName + ".csv")));
                writeLine(polledData.keySet().toArray(new String[polledData.keySet().size()]));
            } catch (IOException e) {
                //DO SOMETHING?
            }

            run = true;
            start();
        }
    }

    public void run() {
        //POLLS THE HARDWARE MAP, WRITES THE POLLED DATA TO THE CSV FILE, AND SLEEPS FOR THE LENGTH OF THE "pollInterval" VARIABLE
        try {
            while (run) {
                pollHardware();
                 writeLine(polledData.values().toArray());

                 Thread.sleep(0);
                 //Thread.sleep(pollInterval);
            }

            bufferedWriter.close();
        } catch (InterruptedException | IOException e) {
            //DO SOMETHING?
        }
    }


    public void pollHardware() {
        //POLLS ALL OF THE HARDWARE DEVICES IN THE HARDWARE MAP AND ADDS THE DATA TO THE "polledData" HASMAP
        String name = "";
        HardwareDevice device = null;

        polledData.put("Time (ms)", System.currentTimeMillis() - startTime);
        polledData.put("Current OpMode", currentOpModeName);
        polledData.put("Odometry X", Odometry.getX());
        polledData.put("Odometry Y", Odometry.getY());
        polledData.put("Odometry Gyro", Odometry.getGyro());

        for (int i = 0; i < polledHardware.length; i++) {
            device = polledHardware[i];
            name = device.getConnectionInfo() + device.getClass().getSimpleName();//.replaceFirst("com.qualcomm.", "");// + "-" + device.getDeviceName();

            if (device instanceof DcMotor) {
                DcMotor motor = (DcMotor) device;
                polledData.put(name + "_Power", motor.getPower());
                polledData.put(name + "_CurrentPosition", motor.getCurrentPosition());
                //polledData.put(name + "_Direction", motor.getDirection());
                polledData.put(name + "_ZeroPowerBehavior", motor.getZeroPowerBehavior());
                polledData.put(name + "_Mode", motor.getMode());
                //polledData.put(name + "_IsBusy", motor.isBusy());
                //polledData.put(name + "_TargetPosition", motor.getTargetPosition());

            } else if (device instanceof Servo) {
                Servo servo = (Servo) device;
                polledData.put(name + "_Position", servo.getPosition());
                polledData.put(name + "_Direction", servo.getDirection());

            } else if (device instanceof CRServo) {
                CRServo servo = (CRServo) device;
                polledData.put(name + "_Power", servo.getPower());
                polledData.put(name + "_Direction", servo.getDirection());

            } else if (device instanceof ModernRoboticsI2cGyro) {
                ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) device;
                polledData.put(name + "_IsCalibrating", gyro.isCalibrating());
                polledData.put(name + "_Heading", gyro.getHeading());
                polledData.put(name + "_AngularVelocity", gyro.getAngularVelocity(AngleUnit.DEGREES));

            } else if (device instanceof ModernRoboticsI2cRangeSensor) {
                ModernRoboticsI2cRangeSensor rangeSensor = (ModernRoboticsI2cRangeSensor) device;
                polledData.put(name + "_Distance", rangeSensor.getDistance(DistanceUnit.INCH));
                //polledData.put(name + "_LightDetected", rangeSensor.getLightDetected());
                polledData.put(name + "_RawOptical", rangeSensor.rawOptical());
                polledData.put(name + "_RawUltrasonic", rangeSensor.rawUltrasonic());

            } else if (device instanceof Rev2mDistanceSensor) {
                Rev2mDistanceSensor rangeSensor = (Rev2mDistanceSensor) device;
                polledData.put(name + "_Distance", rangeSensor.getDistance(DistanceUnit.INCH));

            } else if (device instanceof ColorSensor) {
                ColorSensor colorSensor = (ColorSensor) device;
                polledData.put(name + "_Red", colorSensor.red());
                polledData.put(name + "_Green", colorSensor.green());
                polledData.put(name + "_Blue", colorSensor.blue());
                polledData.put(name + "_LightDetected", colorSensor.alpha());
                //polledData.put(name + "_ARGB", colorSensor.argb());

            }
        }
    }


    //WRITES A STRING ARRAY (REPRESENTING THE DATA SET OF ONE HARDWARE POLL) IN CORRECT CSV FORMAT TO THE CSV FILE
    private void writeLine(Object[] data) {
        try {
            for (int i = 0; i < data.length; i++) {
                bufferedWriter.append(data[i] + (i == data.length-1 ? "" : ","));
            }
            bufferedWriter.append("\n");
            bufferedWriter.flush();
        } catch (IOException e) {
            //DO SOMETHING?
        }

    }


    public <C extends OpMode> void stopLogging(C callingClass) {
        switch (logMode) {
            case OFF:
                //run = false;
                //break;
            case OPMODE:
                run = false;
                break;
            case MATCH:
                //ONLY STOPS RUNNING IF THE CLASS CALLING THIS METHOD IS A CHILD CLASS OF "OpMode" (ONLY TELEOP)
                if (callingClass.getClass().getSuperclass().equals(OpMode.class)) {
                    run = false;
                }
                break;
        }

        currentOpModeName = "NONE";
    }


    //RETURNS THE "polledData" HASHMAP
    public LinkedHashMap<String, Object> getPolledData() {
        return polledData;
    }

    //SETS THE CURRENT LOG MODE
    public static void setLogMode(LogModes m) {
        logMode = m;
    }

    //RETURNS THE CURRENT LOG MODE
    public static LogModes getLogMode() {
        return logMode;
    }


    //SETS THE CURRENT SAVE FILE PATH FOR THE CSV FILES
    public static void setSaveFilePath(String path) {
        saveFilePath = path;
    }

    //RETURNS THE CURRENT SAVE FILE PATH FOR THE CSV FILES
    public static String getSaveFilePath() {
        return saveFilePath;
    }


    //SETS THE POLLING INTERVAL FOR THE DATA LOGGER
    public static void setPollInterval(long interval) {
        pollInterval = interval;
    }

    //RETURNS THE POLLING INTERVAL FOR THE DATA LOGGER
    public static long getPollInterval() {
        return pollInterval;
    }

}
