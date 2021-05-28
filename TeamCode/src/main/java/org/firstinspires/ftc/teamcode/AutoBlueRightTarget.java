package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto//Blue//Right//Target")
public class AutoBlueRightTarget extends LinearOpMode {
    Robot2020 R = new Robot2020();

    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        R.initHWD(telemetry, hardwareMap, this);
        R.init();
        R.VuforiaInit();
        R.initDashboard(FtcDashboard.getInstance());

        R.UP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.UP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        R.servoPull.setPosition(0.73);
        R.servoWobble.setPosition(0.7);

        telemetry.addData("Mode", "running");
        telemetry.update();

        Bitmap bm = R.getImage();
        position = R.colorAnalyser(bm);

        telemetry.addData("Position", position);
        telemetry.update();

        R.GoTo(0, 2.56, R.Accuracy);

        R.delay(200);

        R.holdLift.start();
        R.sh = 2;
        R.delay(3000);

        R.pushRing();

        R.turnToMin(-6.);
        R.delay(300);
        R.pushRing();

        R.turnToMin(-12.);
        R.delay(300);
        R.pushRing();

        R.flagLift = false;
        R.sh = 0;

        R.delay(500);
        if (position == 3) {

            R.RobotAngle = 45;
            R.GoTo(-1.3, 4.5, R.Fast);

            R.wobbleAuto();

            R.GoTo(0.4,2.8, R.Fast);
        }
        else if (position == 2){
            R.RobotAngle = 45;
            R.GoTo(-0.2, 3.5, R.Fast);

            R.wobbleAuto();

            R.GoTo(0, 2.8, R.Fast);
        }
        else {
            R.servoWobble.setPosition(0.7);
            R.delay(500);

            R.turnTo(45);
            R.GoTo(0, 3.2, R.Fast);
            R.turnTo(90);

            R.GoTo(-1, 3.2, R.Fast);

            R.wobbleAuto();

            R.GoTo(0, 3.0, R.Fast);
        }

        R.turnTo(0);
    }
}
