/*
Child class of Move. Constance addition methods that are only used in autos
*/

package org.firstinspires.ftc.teamcode.Objects;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Exceptions.InvalidUnitsException;
import org.firstinspires.ftc.teamcode.Static.Odometry;
import org.firstinspires.ftc.teamcode.Static.Units;

public class Auto extends Move {
    private Character targetZone = ' ';
    private long startTime;

    public Auto(RobotHardware r) {
        super(r);
        startTime = System.currentTimeMillis();
    }
    public Auto(RobotHardware r, LinearOpMode o) {
        super(r, o);
        startTime = System.currentTimeMillis();
    }


    public double measureStartPosition() {
        return ((GameField.fieldWidth * (2.0/3.0)) - ((GameField.alliance == GameField.Alliances.RED ? robot.rightPositionSensor : robot.leftPositionSensor).getDistance(DistanceUnit.INCH) + (GameField.robotWidth / 2))) - 24;
    }

    public void shootPowerShotTargets() {
        for (int powerShot = 3; powerShot < 6; powerShot++) {
            //Shooter.aim(powerShot);
            //Shooter.shoot();
        }
    }

    public void measureRingStack() {
        double measuredHeight;

        measuredHeight = (robot.leftRingSensor.getDistance(DistanceUnit.INCH) < robot.rightRingSensor.getDistance(DistanceUnit.INCH) ? robot.leftRingSensor : robot.rightRingSensor).getDistance(DistanceUnit.INCH);

        if (measuredHeight < (GameField.starterStackHeight[0] + GameField.starterStackHeight[1]) / 4 * 3) {
            targetZone = 'C';
        } else {
            if (measuredHeight < (GameField.starterStackHeight[1] + GameField.starterStackHeight[2]) / 2) {
                targetZone = 'B';
            } else {
                targetZone = 'A';
            }
        }
    }

    public Character getTargetZone() {
        return targetZone;
    }

    public void driveToLaunchLine(double power, double angle, double turnPower) {
        double time = System.currentTimeMillis();
        while (robot.colorSensor.blue() < (GameField.fieldColor.get("Blue") + GameField.launchLineColor.get("Blue")) / 2 && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            drive(power, angle, turnPower);
            Odometry.run(robot);
        }
        stop();
    }
    public void driveToLaunchLinePID(double x, double y, double turnPower) {
        double time = System.currentTimeMillis();
        while (robot.colorSensor.blue() < (GameField.fieldColor.get("Blue") + GameField.launchLineColor.get("Blue")) / 2 && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            driveToPosition(x, y, turnPower);
        }
        stop();
    }

    public void driveToLaunchLine(double power) {
        double time = System.currentTimeMillis();
        while (robot.colorSensor.blue() < (GameField.fieldColor.get("Blue") + GameField.launchLineColor.get("Blue")) / 2 && ((opMode != null) ? (opMode.opModeIsActive()) : true) && System.currentTimeMillis()-time < 3000) {
            drive(power, 0);
        }
        stop();
    }


    //public void waitUntil(int seconds) {
    //    SystemClock.sleep(startTime + (long) Units.convert(seconds, Units.ValidUnits.SECOND, Units.ValidUnits.MILLISECOND) - System.currentTimeMillis());
    //}

    //public float getTime() {
    //    return Units.toSeconds( 0, System.currentTimeMillis() - startTime);
    //}

    //public double getRemainingTime() {
    //    return (30 - Units.convert(System.currentTimeMillis(), Units.ValidUnits.MILLISECOND, Units.ValidUnits.SECOND));
    //}
}