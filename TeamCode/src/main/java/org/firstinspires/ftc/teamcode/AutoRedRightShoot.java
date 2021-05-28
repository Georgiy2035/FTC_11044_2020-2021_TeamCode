package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(name = "Auto//Red//Right//Shoot")

public class AutoRedRightShoot extends LinearOpMode {
    Robot2020 R = new Robot2020();

    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        R.initHWD(telemetry, hardwareMap, this);
        R.init();
        R.initDashboard(FtcDashboard.getInstance());
        R.VuforiaInit();

        R.UP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.UP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.SH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        R.servoPull.setPosition(0.73);
        R.servoWobble.setPosition(0.68);

        telemetry.addData("Mode", "running");
        telemetry.update();

        Bitmap bm = R.getImage();
        position = R.colorAnalyser(bm);

        telemetry.addData("Position", position);
        telemetry.update();

        if (position == 3 || position == 2) {
            R.holdLift.start();

            R.GoTo(-0.45, 1.1, R.Accuracy);

            R.sh = 1;

            R.delay(1700);
        }
        else {
            R.GoTo(-1, 2.56, R.Accuracy);

            R.delay(200);

            R.holdLift.start();
            R.sh = 2;
            R.delay(2000);

            R.pushRing();

            R.turnToMin(6.2);
            R.delay(300);
            R.pushRing();

            R.turnToMin(12.2);
            R.delay(300);
            R.pushRing();

            R.flagLift = false;
            R.sh = 0;
        }
        if (position == 3) {
            R.pushRing();
            R.delay(500);
            R.pushRing();
            R.delay(500);
            R.pushRing();

            R.flagLift = false;
            R.RA.fCatch = 1;
            R.sh = 0;

            R.GoTo(-0.41, 1.35, R.Slow);
            R.GoTo(-0.41, 1.2, R.Slow);
            R.GoTo(-0.41, 1.49, R.Slow);
            R.sh = 1;
            R.delay(1000);

            R.RA.fCatch = 0;
            R.flagLift = true;

            R.delay(3000);

            for (int i = 0; i < 3 && !isStopRequested(); i++) {
                R.delay(500);
                R.pushRing();
            }
            R.flagLift = false;
            R.RA.fCatch = 1;
            R.sh = 0;

            R.GoTo(-0.41, 1.35, R.Slow);
            R.GoTo(-0.41, 1.59, R.Slow);
            R.GoTo(-0.41, 1.45, R.Slow);
            R.GoTo(-0.41, 1.7, R.Slow);
            R.sh = 1;
            R.delay(1700);

            R.RA.fCatch = 0;
            R.flagLift = true;

            R.delay(1500);

            for (int i = 0; i < 3 && !isStopRequested(); i++) {
                R.delay(500);
                R.pushRing();
            }R.sh = 0;

            R.flagLift = false;

            R.RobotAngle = -30;
            R.GoTo(0.5, 4.4, R.Fast);

            R.wobbleAuto();

            R.RobotAngle = 0;
            R.GoTo(0.4, 2.8, R.Fast);
        }
        else if (position == 2)
        {
            for (int i = 0; i < 3 && !isStopRequested(); i++) {
                R.delay(500);
                R.pushRing();
            }
            R.flagLift = false;
            R.RA.fCatch = 1;

            R.GoTo(-0.41, 1.35, R.Slow);
            R.GoTo(-0.41, 1.2, R.Slow);
            R.GoTo(-0.41, 1.49, R.Slow);
            R.delay(900);

            R.RA.fCatch = 0;
            R.flagLift = true;

            R.delay(1300);

            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;
            R.sh = 0;

            R.turnTo(10);
            R.GoTo(-0.45, 3.4, R.Fast);

            R.wobbleAuto();

            R.GoTo(-0.45, 2.8, R.Fast);
        }
        else
        {
            R.flagLift = false;
            R.sh = 0;

            R.turnTo(-90);

            R.GoTo(0.1, 3, R.Fast);

            R.wobbleAuto();

            R.GoTo(-1, 3, R.Fast);
            R.turnTo(0);

        }
    }
}
