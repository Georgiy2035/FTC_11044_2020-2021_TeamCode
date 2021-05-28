package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto//Red//Right//Vobbl")
public class AutoRedRightVobbl extends LinearOpMode {
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
            R.GoTo(0, 3, R.Fast);

            R.wobbleAuto();

            R.GoTo(0, -0.9, R.Fast);
        }
        else if (position == 2){
            R.GoTo(0, 2.4, R.Fast);

            R.delay(1000);

            R.turnTo(50);
            R.wobbleAuto();

            R.turnTo(-50);
            R.delay(1000);

            R.GoTo(0, -0.5, R.Fast);
        }
        else{
            R.GoTo(0, 1.6, R.Fast);
            R.wobbleAuto();
            R.turnTo(-20);

            R.VB.setPower(0.5);
            R.delay(1000);
            R.VB.setPower(0);
        }
    }
}
