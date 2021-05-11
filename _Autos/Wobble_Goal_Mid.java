/*
This auto reads the ring stack
launches 3 rings into mid goal
places wobble goal
parts on launch line
 */
package org.firstinspires.ftc.teamcode._Autos;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.Driver;
import org.firstinspires.ftc.teamcode.Objects.Auto;
import org.firstinspires.ftc.teamcode.Objects.DataLogger;
import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Objects.Shooter;
import org.firstinspires.ftc.teamcode.Static.Controls;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Static.Odometry;


@Autonomous
public class Wobble_Goal_Mid extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Auto move;
    RobotHardware robot;
    Shooter gun;
    DataLogger dataLogger;

    public double stackHeight;


    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        move = new Auto(robot, this);
        gun = new Shooter(robot, move, this);
        dataLogger = new DataLogger(hardwareMap);
        Odometry.reset(robot);
        Odometry.restGyro(robot);
        Odometry.setPosition(move.measureStartPosition(), GameField.robotLength / 2);

        move.grabWobbleGoal();

        telemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Distance", move.measureStartPosition());
        telemetry.addData("X", Odometry.getX());
        telemetry.addData("Y", Odometry.getY());
        telemetry.update();
        waitForStart();
        dataLogger.startLogging(this);
        robot.LEDs.setPower(0);
        gun.setCurrentPower(0.4);






        if (this.opModeIsActive()) {
            //Move away from the wall
            move.driveToPosition(GameField.starterStackX, 30, 0);
            //Line robot up with ring stack
            move.driveToPosition(GameField.starterStackX, GameField.starterStackY - GameField.robotLength /2 -3*GameField.ringRadius, 0);
            //Move to ring stack
            move.driveToPositionAccurate(GameField.starterStackX+GameField.ringRadius, GameField.starterStackY - GameField.robotLength /2 -GameField.ringRadius, 0, 1.5);

            //Measure the height of the ring stack
            SystemClock.sleep(500);
            move.measureRingStack();
            SystemClock.sleep(200);
            telemetry.addData("Target Zone", "    " + move.getTargetZone());
            telemetry.update();

            //Line up with High Goal
            move.driveToPositionAccurate(GameField.starterStackX-3*GameField.ringRadius-GameField.robotLength, GameField.starterStackY - GameField.robotLength /2 -GameField.ringDiameter, 0, 1.5);
            move.driveToPositionAccurate(GameField.starterStackX-3*GameField.ringRadius-GameField.robotLength, GameField.starterStackY + GameField.robotLength /2, 0, 1.5);
            move.driveToPosition(GameField.goalX[2]+8, GameField.starterStackY + GameField.robotLength /2, -10/******************/);

            //Shoot rings
            gun.spinFlywheel(1);
            for (int i = 0; i < 3; i++) {
                SystemClock.sleep(1000);
                gun.shoot();
            }
            gun.spinFlywheel(0);

            //Move to correct Target Zone
            move.driveToPosition(GameField.targetZoneX.get( move.getTargetZone() ) -GameField.tagetZoneWidth/3,GameField.targetZoneY.get( move.getTargetZone() )-GameField.tagetZoneWidth*3/4, -135);


            //Place Wobble Goal
            move.setArm(180);
            move.releaseWobbleGoal();
            move.setArm(40);

            //Move array from Target Zone
            move.driveToPosition(GameField.targetZoneX.get( move.getTargetZone() ) -GameField.tagetZoneWidth/2 -GameField.wobbleGoalRadius,GameField.targetZoneY.get( move.getTargetZone() )-GameField.tagetZoneWidth/2 -GameField.wobbleGoalRadius, -135);
            //Turn to 0 degrees
            move.driveToPosition(GameField.targetZoneX.get( move.getTargetZone() ) -GameField.tagetZoneWidth/2 -GameField.wobbleGoalRadius,GameField.targetZoneY.get( move.getTargetZone() )-GameField.tagetZoneWidth/2 -GameField.wobbleGoalRadius, 0);

            //Line up with targets
            move.driveToPosition(GameField.targetZoneX.get( 'B' ), GameField.targetZoneY.get( move.getTargetZone() )-GameField.tagetZoneWidth -GameField.wobbleGoalDiameter, 0);
            //park on the launch line
            move.driveToPositionAccurate(GameField.targetZoneX.get( 'B' ),GameField.launchLineY , 0, 3);


            Odometry.toTele();
        }

        move.stop();


        telemetry.addData("All Done", "");
        telemetry.update();
        dataLogger.stopLogging(this);
        while (opModeIsActive()) {
        }
    }


}