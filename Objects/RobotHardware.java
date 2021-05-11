/*
Stores all the code for initialization.
 */

package org.firstinspires.ftc.teamcode.Objects;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drivers.Driver;

public class RobotHardware {
    private HardwareMap hardwareMap;

    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cRangeSensor leftRingSensor = null;
    public ModernRoboticsI2cRangeSensor rightRingSensor = null;

    public Rev2mDistanceSensor leftPositionSensor = null;
    public Rev2mDistanceSensor rightPositionSensor = null;
    public ColorSensor colorSensor = null;
    public DcMotor[] motor = new DcMotor[4];
    public DcMotor intake = null;
    public DcMotor shooter1 = null;
    public DcMotor shooter2 = null;

    public CRServo rightArmServo = null;
    public CRServo leftArmServo = null;
    public Servo grabServo = null;
    public Servo ringFlickerServo = null;

    public ModernRoboticsI2cGyro armServoGyro = null;

    public DcMotor LEDs = null;
    //public Servo armServo

    //public Servo testLinearServo = null;

    public void init(HardwareMap hardwareMap) {
        
        rightArmServo = hardwareMap.crservo.get("rightArmServo");
        leftArmServo = hardwareMap.crservo.get("leftArmServo");
        leftArmServo.setDirection(DcMotorSimple.Direction.REVERSE);

        grabServo = hardwareMap.servo.get("grabServo");
        ringFlickerServo = hardwareMap.servo.get("flicker");


        //testLinearServo = hardwareMap.servo.get("testLinearServo");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        armServoGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("armServoGyro");

        leftRingSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRingSensor");
        rightRingSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRingSensor");

        leftPositionSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftPositionSensor");
        rightPositionSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightPositionSensor");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);



        for(int i = 0; i<4; i++){
            motor[i] = hardwareMap.dcMotor.get(String.valueOf(i));
            motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor[i].setPower(0);
        }
        /*motor[0] = hardwareMap.dcMotor.get("0");
        motor[1] = hardwareMap.dcMotor.get("1");
        motor[2] = hardwareMap.dcMotor.get("2");
        motor[3] = hardwareMap.dcMotor.get("3");*/

        motor[0].setDirection(DcMotor.Direction.REVERSE);
        motor[1].setDirection(DcMotor.Direction.FORWARD);
        motor[2].setDirection(DcMotor.Direction.FORWARD);
        motor[3].setDirection(DcMotor.Direction.REVERSE);

        LEDs = hardwareMap.get(DcMotor.class, "LEDs");

        gyro.calibrate();
        while (gyro.isCalibrating())  {
            SystemClock.sleep(250);
        }

        /*armServoGyro.calibrate();
        while (armServoGyro.isCalibrating())  {
            SystemClock.sleep(250);
        }*/

        if (Driver.D1 == null) {
            Driver.D1 = 0;
        }
        if (Driver.D2 == null) {
            Driver.D2 = 3;
        }

        LEDs.setPower(1);

    }
}
