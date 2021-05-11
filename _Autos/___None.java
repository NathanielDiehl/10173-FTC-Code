/*
This Auto does absolutely nothing.
It simply reinitializing odometry.
We have this program because sometimes your alliance partner has an incompatible auto. If it is more strategic to use their auto, you can simply run this program during autonomous and your odometry will be set for teleOp.
 */
package org.firstinspires.ftc.teamcode._Autos;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Objects.Auto;
import org.firstinspires.ftc.teamcode.Objects.DataLogger;
import org.firstinspires.ftc.teamcode.Objects.Move;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Static.Odometry;
import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Objects.Shooter;
import org.firstinspires.ftc.teamcode.Static.Units;

@Autonomous
public class ___None extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DataLogger dataLogger;
    double dotSpot = 0;

    Auto move;
    RobotHardware robot;
    Shooter gun;

    @Override
    public void runOpMode() {
        dataLogger = new DataLogger(hardwareMap);

        robot = new RobotHardware();
        robot.init(hardwareMap);
        move = new Auto(robot, this);
        gun = new Shooter(robot, move, this);
        Odometry.reset(robot);
        Odometry.setPosition(move.measureStartPosition(), GameField.robotLength / 2);


        telemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        dataLogger.startLogging(this);

        if(this.opModeIsActive() ) {
        }

        telemetry.update();

        move.stop();
        Odometry.toTele();
        dataLogger.stopLogging(this);
    }


}