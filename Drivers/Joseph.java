/*
This class overloads methods in order to better match Joseph's personal preferences
*/

package org.firstinspires.ftc.teamcode.Drivers;

import org.firstinspires.ftc.teamcode.Objects.RobotHardware;
import org.firstinspires.ftc.teamcode.Static.Controls;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class Joseph extends Driver{


    public Joseph(Gamepad g1, Gamepad g2){
        super(g1, g2, "Joseph");
        log.put("lj", 0.5);
        log.put("rj", 0.5);
        log.put("lt", 0.5);
        log.put("rt", 0.5);

        rightJoyCut = 0.5;
    }


}

