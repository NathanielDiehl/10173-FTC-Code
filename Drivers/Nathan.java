/*
This class overloads methods in order to better match Nathan's personal preferences
*/


package org.firstinspires.ftc.teamcode.Drivers;

import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Static.Controls;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class Nathan extends Driver{

    public Nathan(Gamepad g1, Gamepad g2){
        super(g1, g2, "Nathan");
        log.put("lj", 0.75);
        log.put("rj", 0.75);
        log.put("lt", 0.75);
        log.put("rt", 0.75);
    }

}
