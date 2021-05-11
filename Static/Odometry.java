/*
Uses the output from the two encoder wheels at the bottom of the robot
The code then uses trigonometric functions with the current gyro value to calculate the robot’s position
 */

package org.firstinspires.ftc.teamcode.Static;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Objects.RobotHardware;


public class Odometry {

    private static double x = 0;
    private static double y = 0;

    private static double old1 = 0;
    private static double old2 = 0;

    private static double delta1 = 0;
    private static double delta2 = 0;

    private static double gyro = 0;
    private static double deltaGyro = 0;
    private static double previousGyro = 0;
    private static double oldGyro = 0;

    private static double encodeInRotation = 0;

    public static void setUp(){
        old1 = 0;
        old2 = 0;
        delta1 = 0;
        delta2 = 0;
    }


    public static void run(RobotHardware robot) {

        gyro = -(Math.toRadians(robot.gyro.getIntegratedZValue()) + previousGyro);
        deltaGyro = (gyro - oldGyro);
        oldGyro += deltaGyro;

        delta1 = (-robot.motor[0].getCurrentPosition() - old1 + deltaGyro * 3440/1085);
        old1 += delta1;
        delta2 = (robot.motor[1].getCurrentPosition()  - old2 - deltaGyro * 5756/1085);
        old2 += delta2;


        x += /*(delta1 * Math.cos(gyro) + delta2 * Math.sin(gyro) ) *252.14444;*/Units.convert((delta1 * Math.cos(gyro) + delta2 * Math.sin(gyro)), Units.ValidUnits.TICK, Units.ValidUnits.INCH);
        y += /*(delta1 * Math.sin(-gyro) + delta2 * Math.cos(-gyro) ) *252.14444;*/Units.convert((delta1 * Math.sin(-gyro) + delta2 * Math.cos(-gyro)), Units.ValidUnits.TICK, Units.ValidUnits.INCH);
    }

    public static void reset(RobotHardware robot){
        robot.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setUp();
        setPosition(0, 0);
        restGyro(robot);
    }


    public static void setPosition(double newX, double newY){
        x = newX;
        y = newY;
    }

    public static double getGyro () {
        return gyro;
    }
    public static double getX () {
        return x;
    }
    public static double getY () {
        return  y;
    }

    public static void restGyro(RobotHardware robot) {
        oldGyro = 0;
        gyro = 0;
        previousGyro = 0;
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            SystemClock.sleep(50);
        }
    }
    public static void toTele(){
        previousGyro = gyro;
        setUp();
    }

    public static double distanceToTarget(int target){
        return Math.sqrt(Math.pow(x - GameField.goalX[target], 2) + Math.pow(y - GameField.goalY[target], 2));
    }

    public static double angleToTarget(int target){
        return Math.atan(y - GameField.goalY[target] / x - GameField.goalX[target]);
    }
}
