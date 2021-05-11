/*
The Shoot class stores method for all types of ring movement (launching, intaking, and indexing).
By storing the methods in this class, we avoid rewrite the code every time we need to use it.
Instead we can simple reference this class every time we need to use a method.
 */

package org.firstinspires.ftc.teamcode.Objects;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Exceptions.InvalidUnitsException;
import org.firstinspires.ftc.teamcode.Static.Controls;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Static.Odometry;
import org.firstinspires.ftc.teamcode.Static.Units;

import java.lang.Math;

public class Shooter {

    private final double gravity = -385.82698;

    private double launchHeight = 0;
    //private final static double velocityMag = 0;
    private final static double velocityAng = 0;

    private double errorMarginZ = 0.1;
    private double errorMarginX = 0.1;

    private RobotHardware robot;
    private Move move;
    private LinearOpMode opMode;


    private double currentPower = 0;

    public PID rpmController = new PID(0.0000005, 0*0.0000000001, 0, 300, 1);


    public Shooter(RobotHardware r, Move m){
        robot = r;
        move = m;
    }
    public Shooter(RobotHardware r, Move m, LinearOpMode o){
        robot = r;
        move = m;
        opMode = o;
    }

    public double getRPM() {
         double tps = 0;
         long startTime = System.currentTimeMillis();
         double startPosition1 = robot.shooter1.getCurrentPosition();
         double startPosition2 = robot.shooter2.getCurrentPosition();

         tps = (robot.shooter1.getCurrentPosition() - startPosition1 + robot.shooter2.getCurrentPosition() - startPosition2) / (System.currentTimeMillis() - startTime) * 5000;

         rpmController.addToAverage(tps);
         return tps;


    }

    //public void aim(int goalNum) {
    //    while (Math.sin(Math.abs(getTurretRotation(goalNum) - 0/*actualRotation*/)) < errorMarginX && Math.abs(getLaunchAngle(getDistance(goalNum), goalNum) - 0/*actualLaunchAngle*/) < 0/*angleErrorMargin*/) {
    //        //code
    //    }
    //}

    public void shoot() {
        robot.ringFlickerServo.setPosition(.45);
        SystemClock.sleep(350);
        robot.ringFlickerServo.setPosition(0);
        SystemClock.sleep(450);
    }

    public void spinFlywheel(boolean spin) {
            robot.shooter1.setPower(spin? 1:0);
            robot.shooter2.setPower(spin? 1:0);
    }

    public void spinFlywheel(double s) {

        robot.shooter1.setPower(s);
        robot.shooter2.setPower(s);
    }

    public void spinFlywheelPID(double desiredTPS)  {
        currentPower += rpmController.run(getRPM(), desiredTPS);
        robot.shooter1.setPower(Math.abs(currentPower));
        robot.shooter2.setPower(Math.abs(currentPower));
    }
    public void spinFlywheelToSpeed(double desiredRPM) {
        while (rpmController.shouldRun() && ((opMode != null) ? (opMode.opModeIsActive()) : true)) {
            this.spinFlywheelPID(desiredRPM);
        }
    }
    public void autoIntake() {

        robot.shooter1.setPower(1);
        robot.shooter2.setPower(1);
        robot.intake.setPower(1);
        SystemClock.sleep(750);

        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
        robot.intake.setPower(0);
    }
    public void intakeWithEncoder(double en) { intakeWithEncoder(0.5f, en); }
    public void intakeWithEncoder(double power, double en) {
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double start = Math.abs(robot.intake.getCurrentPosition());
        double time = System.currentTimeMillis();

        while (Math.abs(robot.intake.getCurrentPosition()) - start < en && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            robot.intake.setPower(power);
        }

        robot.intake.setPower(0);
    }

    public void intakeUp(boolean spin) {
            robot.intake.setPower(spin? 1:0);
    }

    public void intake(double spin) {
        robot.intake.setPower(spin);
    }

    public void intakeDown(boolean spin) {
        robot.intake.setPower(spin? -1:0);
    }

    /*public void autoAim(int goalNum) {
         auto aims the launch angle
        while (Math.sin(Math.abs(calcTurretRotation(goalNum) - robot.turretGyro.getIntegratedZValue)) < errorMarginX && Math.abs(calcLaunchAngle(calcDistance(goalNum), goalNum) - robot.turretGyro.getIntegratedXHeading) < angleErrorMargin && ((opMode != null) ? (opMode.opModeIsActive()) : true)) {
            //code
        }
    }*/
    /*public void manualAim() {
        //code
    }*/
    /*private float calcTurretRotation(int goalNum) {
        return (float) (Controls.getAngle((GameField.goalX[goalNum] - Odometry.getX()), (GameField.goalY[goalNum] - Odometry.getY())));
    }*/
    /*private float calcDistance(int goalNum) {
        return (float) (Controls.getPower((GameField.goalX[goalNum] - Odometry.getX()), (GameField.goalY[goalNum] - Odometry.getY())));
    }*/
    /*
    private double calcLaunchAngle(double distance, int goalNum) {
        double velocityAng = Math.PI / 4, angleRange = Math.PI / 4;

        double time = distance/(velocityMag*Math.cos(velocityAng));
        double yDist = launchHeight + (velocityMag * Math.sin(velocityAng)) * time + 0.5 * gravity * Math.pow(time, 2);
        double error = yDist - GameField.goalZ[goalNum];

        while (Math.abs(error) > errorMarginZ/*(GameField.goalZ[goalNum]-ringHeight)*/ /*&& velocityAng > 0 && velocityAng < 1.570796326794) { //(goalLevel[goalNum][1]) && velocityAng > 0.0 && velocityAng < Math.PI/2) {
            time = distance / (velocityMag * Math.cos(velocityAng));
            yDist = launchHeight + (velocityMag * Math.sin(velocityAng)) * time + 0.5 * gravity * Math.pow(time, 2);
            error = yDist - GameField.goalZ[goalNum];

            if (error > 0.0) {
                velocityAng -= angleRange / 2;
            } else {
                velocityAng += angleRange / 2;
            }
            angleRange /= 2;
        }
        return velocityAng;
    }
    */
    public double calcLaunchSpeed(double distance, int goalNum) {
        double velocityMag = 250, magRange = 250; //Use half of our possible speed

        double time = distance/(velocityMag*Math.cos(Math.toRadians(velocityAng)));
        double yDist = launchHeight + (velocityMag * Math.sin(Math.toRadians(velocityAng))) * time + 0.5 * gravity * Math.pow(time, 2);
        double error = yDist - GameField.goalZ[goalNum];
        double timer = System.currentTimeMillis();

        while (Math.abs(error) > errorMarginZ && velocityMag > 0 && velocityMag < 500 && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-timer < 3000) { //(goalLevel[goalNum][1]) && velocityAng > 0.0 && velocityAng < Math.PI/2) {
            time = distance / (velocityMag * Math.cos(Math.toRadians(velocityAng)));
            yDist = launchHeight + (velocityMag * Math.sin(Math.toRadians(velocityAng))) * time + 0.5 * gravity * Math.pow(time, 2);
            error = yDist - GameField.goalZ[goalNum];
            System.out.println(velocityMag);
            System.out.println(yDist);

            if (error > 0.0) {
                velocityMag -= magRange / 2;
            } else {
                velocityMag += magRange / 2;
            }
            magRange /= 2;
        }
        return velocityMag;
    }
    /*public void setErrorMargins(float X, float Z) {
        errorMarginX = X;
        errorMarginZ = Z;
    }*/

    public double getCurrentPower() {
        return currentPower;
    }

    public void setCurrentPower(double cp) {
        currentPower = cp;
    }
}
