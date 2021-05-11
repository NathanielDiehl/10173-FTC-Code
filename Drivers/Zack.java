/*
This class overloads methods in order to better match Nathan's personal preferences
*/


package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Static.Controls;

import java.util.HashMap;

public class Zack extends Driver{


    public Zack(Gamepad g1, Gamepad g2){
        super(g1, g2, "Zack");
        leftJoyCut = 0.65;
        Augmented = false;
    }

    public double power(){
        return (Math.pow(Controls.getPower(gamepad1.left_stick_x, gamepad1.left_stick_y), log.get("lj") ) * (1 - gamepad1.left_trigger * 0.45) );
    }

}
