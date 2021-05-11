/*
This Auto sets the GameField variables from red alliance
 */
package org.firstinspires.ftc.teamcode._Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Static.GameField;

@Autonomous
public class __Red extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        GameField.setAlliance(GameField.Alliances.RED);
    }
}