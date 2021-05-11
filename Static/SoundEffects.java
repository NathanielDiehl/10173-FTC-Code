/*
Plays sound effects on the robot controller phone.
Only works on the robot controller phone
 */
package org.firstinspires.ftc.teamcode.Static;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.media.MediaPlayer;

public class SoundEffects {
    private static LinearOpMode opMode;
    SoundEffects(LinearOpMode opMode){
        this.opMode = opMode;
    }
    SoundEffects(){
    }
    private MediaPlayer mp = new MediaPlayer();
    //private ToggleUtility mediaPlay = new ToggleUtility();
    private final String mediaPath = "/storage/emulated/0/Music";
    private String mediaFile;

    public void setSound(String file){
        mediaFile = file;
    }

    public void playSound() {
        try {
            mp.setDataSource(mediaPath + "/" + mediaFile);
            mp.prepare();
            mp.start();
        } catch(Exception e){
            try {
                opMode.telemetry.addData("Failed", "playing sound");
                opMode.telemetry.update();
            } catch(Exception f){
            }
        }

    }
}
