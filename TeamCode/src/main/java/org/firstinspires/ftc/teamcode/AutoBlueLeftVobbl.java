package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto//Blue//Left//Vobble")
public class AutoBlueLeftVobbl extends LinearOpMode{

    Robot2020 R = new Robot2020();

    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        R.initHWD(telemetry, hardwareMap, this);
        R.init();
        R.VuforiaInit();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        Bitmap bm = R.getImage();
        position = R.colorAnalyser(bm);

        if (position == 3) {
            R.GoTo(0, 2.7, R.Fast);
            R.turnTo(50);

            R.wobbleAuto();

            R.turnTo(-50);
            R.GoTo(0, -1, R.Fast);
        }
        else if (position == 2){
            R.GoTo(0, 1.8, R.Fast);

            R.delay(1000);

            R.turnTo(-18);

            R.GoTo(0, 0.65, R.Fast);

            R.wobbleAuto();
            //R.turnTo(-50);
            R.delay(1000);

            R.GoTo(0, -0.5, R.Fast);
        }
        else {
            R.GoTo(0, 1.6, R.Fast);
            R.turnTo(50);

            R.wobbleAuto();
            R.turnTo(-70);
            R.GoTo(0, 0.4, R.Fast);
        }
    }
}
