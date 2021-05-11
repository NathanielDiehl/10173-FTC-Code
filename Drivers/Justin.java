/*
This class overloads methods in order to better match Justin's personal preferences
*/


package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Static.Controls;

import java.util.HashMap;

public class Justin extends Driver{

    public Justin(Gamepad g1, Gamepad g2){
        super(g1, g2, "Justin");
        rightJoyCut = 0.4;
    }


}
