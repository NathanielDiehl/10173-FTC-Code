/*
This class stores default methods for every control.
This class is the parent class to each of our personalized driver classes.
The child classes (Joseph, Justin, Nathan, Troy, Zack) overload methods in order to better match personal preferences
 */
package org.firstinspires.ftc.teamcode.Drivers;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Static.Controls;

import java.util.HashMap;

public class Driver {

    public static Integer D1;
    public static Integer D2;

    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    protected String name = "";
    protected double leftJoyCut = 0.5;
    protected double rightJoyCut = 0.2;
    protected boolean Augmented = true;
    protected static HashMap<String, Double> log = new HashMap<String, Double>() {{
        put("lj", 1.0);
        put("rj", 1.0);
        put("lt", 1.0);
        put("rt", 1.0);
    }};


    public Driver(Gamepad g1, Gamepad g2, String n){
        gamepad1 = g1;
        gamepad2 = g2;
        name = n;
    }

    public String getName(){
        return name;
    }

    public static void changeDriver(Driver d1, Driver d2, Driver[] driver) {
        if (d1.gamepad1.start && d1.gamepad1.dpad_right) {
            Driver.D1 = (Driver.D1 < (driver.length-1)) ? Driver.D1 + 1 : 0;
            SystemClock.sleep(250);
        } else if (d1.gamepad1.start && d1.gamepad1.dpad_left) {
            Driver.D1 = (Driver.D1 > 0) ? Driver.D1 - 1 : (driver.length - 1);
            SystemClock.sleep(250);
        }

        if(d2.gamepad2.start && d2.gamepad2.dpad_right) {
            Driver.D2 = (Driver.D2 < (driver.length-1) ) ? Driver.D2+1 : 0;
            SystemClock.sleep(250);
        } else if(d2.gamepad2.start && d2.gamepad2.dpad_left) {
            Driver.D2 = (Driver.D2 > 0 ) ? Driver.D2-1 : (driver.length - 1);
            SystemClock.sleep(250);
        }
    }

    public void setAugmented(boolean a){
        Augmented = a;
    }

    public boolean getAugmented(){
        return Augmented;
    }

    /***************************Gamepad1***************************/
    public boolean braking(){
        return gamepad1.right_bumper;
    }
    public boolean augmentedSwitch(){
        return gamepad1.y && gamepad1.x;
    }
    public boolean driveToSpot(){
        return gamepad1.a;
    }

    public boolean resetGyro(){
        return gamepad1.a && gamepad1.b;
    }

    public double power(){
        return (Math.pow( Controls.getPower(gamepad1.left_stick_x, gamepad1.left_stick_y), log.get("lj") ) * (leftJoyCut + Math.pow(gamepad1.left_trigger, log.get("lt") *(1-leftJoyCut)) ));
    }
    public double angle(){
        return Controls.getAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
    }
    public double turning(){
        return (Math.pow(gamepad1.right_stick_x, log.get("rj") )*( Math.pow(gamepad1.right_trigger*(1-rightJoyCut), log.get("rt") )+rightJoyCut));
    }
    public boolean switchTargetUp(){
        return gamepad1.dpad_right;
    }
    public boolean switchTargetDown(){
        return gamepad1.dpad_left;
    }


    /***************************Gamepad2***************************/
    public boolean autoIntake(){
        return gamepad2.b;
    }
    public boolean intakeUp(){ return gamepad2.dpad_up ; }
    public boolean intakeDown(){ return gamepad2.dpad_down; }


    public boolean shoot(){ return gamepad2.left_trigger > 0; }

    public boolean spinFlywheel(){ return gamepad2.right_trigger > 0; }
    public boolean increaseFlywheelSpeed(){ return gamepad2.y; }
    public boolean decreaseFlywheelSpeed(){ return gamepad2.a; }

    public boolean liftWobbleGoalArm(){ return gamepad2.left_bumper; }
    public boolean lowerWobbleGoalArm(){ return gamepad2.right_bumper ; }
    public boolean WobbleGoalGrab(){ return gamepad2.b ; }

}
