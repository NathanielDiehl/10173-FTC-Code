/*
Driver controlled programming for teleOp mode
Uses 3 threads to increase cycles per second
 */

package org.firstinspires.ftc.teamcode._Tele;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.Joseph;
import org.firstinspires.ftc.teamcode.Drivers.Justin;
import org.firstinspires.ftc.teamcode.Drivers.Nathan;
import org.firstinspires.ftc.teamcode.Drivers.Troy;
import org.firstinspires.ftc.teamcode.Drivers.Zack;
import org.firstinspires.ftc.teamcode.Objects.DataLogger;
import org.firstinspires.ftc.teamcode.Objects.Move;
import org.firstinspires.ftc.teamcode.Objects.Shooter;
import org.firstinspires.ftc.teamcode.Static.Controls;
import org.firstinspires.ftc.teamcode.Static.GameField;
import org.firstinspires.ftc.teamcode.Static.Odometry;
import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Drivers.Driver;
import org.firstinspires.ftc.teamcode.Objects.Auto;

@TeleOp(name="Threads_Tele_Op_Mode ", group="Iterative Opmode")

public class   Threads_Tele_Op_Mode  extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DataLogger dataLogger;

    private double power;
    private double angle;
    private double turning;

    public boolean end;

    private int selectedTarget;

    gamepad_one G1;
    gamepad_two G2;
    odometry odom;

    Move move;
    RobotHardware robot;
    Shooter gun;
    Driver[] driver = new Driver[5];

    double speed;
    double TPSspeed;
    boolean grab;

    @Override
    public void init() {
        dataLogger = new DataLogger(hardwareMap);

        Odometry.setUp();

        //Initialize objects
        robot = new RobotHardware();
        robot.init(hardwareMap);
        move = new Move(robot);
        gun = new Shooter(robot, move);
        G1 = new gamepad_one();
        G2 = new gamepad_two();
        odom = new odometry();

        //Creates objects for each driver
        driver[0] = new Zack(gamepad1, gamepad2);
        driver[1] = new Joseph(gamepad1, gamepad2);
        driver[2] = new Justin(gamepad1, gamepad2);
        driver[3] = new Troy(gamepad1, gamepad2);
        driver[4] = new Nathan(gamepad1, gamepad2);

        //Initialize variables
        power = 0;
        angle = 0;
        turning = 0;
        speed = 1;
        TPSspeed = 1000;
        grab = true;
        end = false;
        selectedTarget = 2;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        dataLogger.startLogging(this);
        //Start threads;
        G1.start();
        G2.start();
        odom.start();
    }


    @Override
    public void loop() {
    //Set drivers to default
        Driver.changeDriver(driver[Driver.D1], driver[Driver.D2], driver);
        updateTelemetry();
    }


    public void updateTelemetry() {
    //Prints data to the phone
        telemetry.update();
        telemetry.addData("Target", "  " + GameField.goalName[selectedTarget]);
        telemetry.addData("RPM  ", gun.getRPM() );
        telemetry.addData("Flywheel Speed  ", speed );
        telemetry.addData("Driver 1", "  " +driver[Driver.D1].getName() + ",  Driver 2 :  " +driver[Driver.D2].getName());
        telemetry.addData("Augmented Driving", "   " +driver[Driver.D1].getAugmented() );
        telemetry.addData("Run Time", "   " + runtime.toString().substring(0, 4) );
    }

    class odometry extends Thread {
    //Runs odometry in designated thread
        odometry() {
        }

        public void run() {
            while (!end) {
                Odometry.run(robot);
            }

        }
    }

    class gamepad_one extends Thread {
    //Runs gamepad one controls in designated thread
        gamepad_one() {
        }

        public void run() {
            while (!end) {
             /***************Setting Variables*****************/
                power = driver[Driver.D1].power();
                angle = driver[Driver.D1].angle();
                turning = driver[Driver.D1].turning();

                if(driver[Driver.D1].switchTargetUp() ) {
                    selectedTarget = (selectedTarget < (GameField.goalName.length - 1) ) ? (selectedTarget+1) : 2;
                    SystemClock.sleep(250);
                } else if(driver[Driver.D1].switchTargetDown() ) {
                    selectedTarget = (selectedTarget > 2 ) ? (selectedTarget-1) : (GameField.goalName.length - 1);
                    SystemClock.sleep(250);
                }

            /*****************Driver Optimization*****************/

                //Braking ON/OFF
                if (driver[Driver.D1].braking()) {
                    move.brakeOff();
                } else {
                    move.brakeOn();
                }

                //Automatically drives to set spot on the field
                if (driver[Driver.D1].driveToSpot()) {
                    move.driveToPositionAccurate(GameField.goalX[selectedTarget], GameField.launchLineY -GameField.robotLength, -6,0.75);
                }

                //Augmented Steering ON/OFF
                if(driver[Driver.D1].augmentedSwitch()) {
                    Controls.switchAugmented(driver[Driver.D1]);
                    SystemClock.sleep(250);
                }

                //reset gyro
                if (driver[Driver.D1].resetGyro() ) {
                    telemetry.addData("Status", "Calibrating");
                    Odometry.reset(robot);
                }

            /*****************Driving*****************/
                if(driver[Driver.D1].getAugmented() ) {
                    move.drive( power, (angle -Odometry.getGyro() -Math.PI/2), turning);
                }else{
                    move.drive(power, angle, turning);
                }

            }
        }
    }

    class gamepad_two extends Thread {
    //Runs gamepad two controls in designated thread
            gamepad_two() {
            }

            public void run() {
                while (!end) {

                    //Changes flywheel speed
                    if(driver[Driver.D2].increaseFlywheelSpeed() ){
                        speed+=0.0125;
                        SystemClock.sleep(250);
                    }else if(driver[Driver.D2].decreaseFlywheelSpeed() ){
                        speed-=0.0125;
                        SystemClock.sleep(250);
                    }

                    //Intake rings
                    if(driver[Driver.D2].intakeUp()){
                        gun.intake(1);
                    } else if(driver[Driver.D2].intakeDown() ) {
                        gun.intake(-1);
                    }else {
                        gun.intake(0);
                    }

                    //Runs flywheel
                    gun.spinFlywheel( (driver[Driver.D2].spinFlywheel()? 1:0) *speed);

                    //Shoots rings
                    if (driver[Driver.D2].shoot() ){
                        gun.shoot();
                    }

                    //Grabs the wobble goal
                    if (driver[Driver.D2].WobbleGoalGrab() ){
                        if (grab) {
                            move.grabWobbleGoal();
                        } else {
                            move.releaseWobbleGoal();
                        }
                        SystemClock.sleep(250);
                        grab = !grab;
                    }

                    //Moves wobble goal arm
                    if (driver[Driver.D2].liftWobbleGoalArm()) {
                        robot.rightArmServo.setPower(-0.65);
                        robot.leftArmServo.setPower(-0.65);
                    } else if (driver[Driver.D2].lowerWobbleGoalArm() ) {
                        robot.rightArmServo.setPower(0.65);
                        robot.leftArmServo.setPower(0.65);
                    }else {
                        robot.rightArmServo.setPower(0.0);
                        robot.leftArmServo.setPower(0.0);

                    }
                }
            }
    }

    @Override
    public void stop() {
        end = true;
        move.stop();
        Odometry.toTele();
        dataLogger.stopLogging(this);
    }

}
