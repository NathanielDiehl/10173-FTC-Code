/*
The Controls class stores method for all types of robot control.
By storing the methods in this class, we avoid rewrite the code every time we need to use it.
Instead we can simple reference this class every time we need to use a method.
 */

package org.firstinspires.ftc.teamcode.Static;


import org.firstinspires.ftc.teamcode.Drivers.Driver;

public class Controls {


    public static double getPower(double left_stick_x, double left_stick_y) {
        return (Math.sqrt(Math.pow(left_stick_y, 2) + Math.pow(left_stick_x, 2)));
    }
    public static double getAngle(double x, double y){
        if (x > 0.05) {
            return (Math.atan(y / x) + (Math.PI / 2));
        } else if (x < -0.05)  {
            return (Math.PI + Math.atan(y / x) + (Math.PI / 2));
        } else if (y > 0) {
            return Math.PI;
        } else {
            return 0;
        }
    }
    public static double getSign(double x) {
        return (x/Math.abs(x) );
    }
    public static void switchAugmented(Driver d){
        d.setAugmented(!d.getAugmented() );
    }
}
