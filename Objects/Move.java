/*
The Move class stores method for all types of robot movement.
By storing the methods in this class, we avoid rewrite the code every time we need to use it.
Instead we can simple reference this class every time we need to use a method.
 */

package org.firstinspires.ftc.teamcode.Objects;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Objects.PID;
import org.firstinspires.ftc.teamcode.Static.Controls;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Static.Odometry;
import org.firstinspires.ftc.teamcode.Objects.RobotHardware;

public class Move {
    static public int GyroAngle = 0;
    protected LinearOpMode opMode;
    protected RobotHardware robot;



    public PID armController = new PID(0.025, 0.0, 0, 3, 1);
    public PID setAngleController = new PID(0.035, 0.000001, 0, 2, 1);
    public PID distanceController = new PID(0.1525, 0.0000725, 0, 1.75, 1);

    public Move (RobotHardware r) {
        this.robot = r;
    }
    public Move (RobotHardware r, LinearOpMode o){
        this.robot = r;
        this.opMode = o;
    }

    public void drive(double power, double angle, double turnPower){
        angle+= (Math.PI / 4);

        double max = 1;

        if(Math.abs(Math.sin( angle)) > Math.abs(Math.sin( angle+ (Math.PI / 2))) ) {
            max = Math.abs( (power-Math.abs(turnPower)) / Math.sin( angle) );
        }else {
            max = Math.abs( (power-Math.abs(turnPower)) / Math.sin( angle+ (Math.PI / 2)) );
        }

        robot.motor[0].setPower( max*power * Math.sin( angle                )  +turnPower);
        robot.motor[1].setPower( max*power * Math.sin( angle+ (Math.PI / 2) )  -turnPower);
        robot.motor[2].setPower( max*power * Math.sin( angle)                  -turnPower);
        robot.motor[3].setPower( max*power * Math.sin( angle+ (Math.PI / 2) )  +turnPower);
    }
    public void drive(double power, double angle){
        drive(power, angle, 0);
    }
    public void driveEn(double power, double angle, double turnPower, double en) {
        this.driveEnNoSet(power, angle, turnPower,en);
        this.setAngle(0);
        this.stop();
    }
    public void driveEnNoSet(double power, double angle, double turnPower, double en) {
        robot.motor[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double start = Math.abs(robot.motor[2].getCurrentPosition());
        double time = System.currentTimeMillis();

        while (Math.abs(robot.motor[2].getCurrentPosition()) - start < en && ((opMode != null) ? (opMode.opModeIsActive()) : true && System.currentTimeMillis()-time < 3000)) {
            Odometry.run(robot);
            if (robot.gyro.getIntegratedZValue() > 0) {
                this.drive(power, angle, turnPower + 0.03);
            } else if (robot.gyro.getIntegratedZValue() < 0) {
                this.drive(power, angle, turnPower - 0.03);
            } else {
                this.drive(power, angle, turnPower);
            }
        }
        this.stop();
    }

    public void driveToPositionWithSpeed(double x, double y, double a, double s) {
        double xOff;
        double yOff;
        double time = System.currentTimeMillis();
        distanceController.calcCurrentError(Controls.getPower(Odometry.getX(), Odometry.getY() ), Controls.getPower(x, y));
        setAngleController.calcCurrentError(a, Math.toDegrees(Odometry.getGyro())%360 );
        while ( (setAngleController.shouldRun() || distanceController.shouldRun() ) && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            Odometry.run(robot);
            xOff = x-Odometry.getX();
            yOff = y-Odometry.getY();

            drive(
                    s*distanceController.run(Controls.getPower(xOff, yOff), 0 ),
                    Controls.getAngle(xOff,-yOff) - Odometry.getGyro(),
                    setAngleController.run(a, Math.toDegrees(Odometry.getGyro() )%360 )
            );

        }
        stop();
        setAngleController.reset();
    }
    public void driveToPosition(double x, double y, double a) {
        driveToPositionWithSpeed(x,y,a,1);
    }
    public void driveToPositionWithSpeedAccurate(double x, double y, double a, double s, double error) {
        double oldError = distanceController.errorMargin;
        distanceController.errorMargin = error;
        driveToPositionWithSpeed(x,y,a,s);
        distanceController.errorMargin = oldError;
    }
    public void driveToPositionAccurate(double x, double y, double a, double error) {
        driveToPositionWithSpeedAccurate(x,y,a,1, error);
    }



    /***********************wobble goal arm control***********************/
    public void setArm(double angle) {
        double time = System.currentTimeMillis();

        //while
        armController.calcCurrentError(robot.armServoGyro.getIntegratedZValue(), angle);
        while (armController.shouldRun() && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            robot.rightArmServo.setPower(-armController.run(robot.armServoGyro.getIntegratedZValue(), angle ) );
            //robot.leftArmServo.setPower(armController.run(robot.armServoGyro.getIntegratedZValue(), angle ) );
        }
        robot.rightArmServo.setPower(0);
        //robot.leftArmServo.setPower(0);

    }

    public void grabWobbleGoal() {
        robot.grabServo.setPosition(0.475);
    }

    public void releaseWobbleGoal() {
        robot.grabServo.setPosition(1);
    }





    /***********************basic control***********************/
    public void brakeOn(){
        for(int i = 0; i<4; i++){
            robot.motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void brakeOff(){
        for(int i = 0; i<4; i++){
            robot.motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


    public void turn_right(double power){
        robot.motor[0].setPower( power);
        robot.motor[1].setPower(-power);
        robot.motor[2].setPower(-power);
        robot.motor[3].setPower( power);
    }
    public void turn_left(double power){
        turn_right(-power);
    }
    public void stop() {
        for(int i = 0; i<4; i++){
            robot.motor[i].setPower(0);
        }
    }
    public void setAngle(double angle) {
        this.driveToPosition(Odometry.getX(),Odometry.getY(), angle);
    }

    public void turnToShootingAngle(int num) {
        setAngle(Controls.getAngle(GameField.goalX[num] - Odometry.getX(), GameField.goalY[num] - Odometry.getY()));
    }
}
