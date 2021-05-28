package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(name = "Auto//Blue//Left//Shoot")

public class AutoBlueLeftShoot extends LinearOpMode {
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

        waitForStart();

        R.servoPull.setPosition(0.73);
        R.servoWobble.setPosition(0.65);

        telemetry.addData("Mode", "running");
        telemetry.update();

        Bitmap bm = R.getImage();
        position = R.colorAnalyser(bm);

        telemetry.addData("Position", position);
        telemetry.update();

        R.holdLift.start();

        R.GoTo(0.41, 1.1, R.Accuracy);

        R.sh = 1;

        R.delay(1800);

        if (position == 3) {
            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;
            R.RA.fCatch = 1;

            R.GoTo(0.41, 1.35, R.Slow);
            R.GoTo(0.41, 1.2, R.Slow);
            R.GoTo(0.41, 1.49, R.Slow);
            R.delay(900);

            R.RA.fCatch = 0;
            R.flagLift = true;

            R.delay(1000);

            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;
            R.RA.fCatch = 1;

            R.GoTo(0.41, 1.35, R.Slow);
            R.GoTo(0.41, 1.59, R.Slow);
            R.GoTo(0.41, 1.45, R.Slow);
            R.GoTo(0.41, 1.74, R.Slow);
            R.delay(1200);

            R.RA.fCatch = 0;
            R.flagLift = true;

            R.delay(900);

            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;

            R.RobotAngle = 30;
            R.GoTo(-0.3, 4.5, R.Fast);

            R.wobbleAuto();

            R.RobotAngle = 0;
            R.GoTo(-0.3, 2.8, R.Fast);
        }
        else if (position == 2)
        {
            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;
            R.RA.fCatch = 1;

            R.GoTo(0.41, 1.35, R.Slow);
            R.GoTo(0.41, 1.2, R.Slow);
            R.GoTo(0.41, 1.49, R.Slow);
            R.delay(1200);

            R.RA.fCatch = 0;
            R.delay(200);
            R.flagLift = true;

            R.delay(1200);

            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;

            R.turnTo(10);
            R.GoTo(0.45, 3.4, R.Fast);

            R.wobbleAuto();

            R.GoTo(0.45, 2.8, R.Fast);
        }
        else
        {
            for (int i = 0; i < 3 && !isStopRequested(); i++)
                R.pushRing();

            R.flagLift = false;

            R.turnTo(90);

            R.GoTo(0.45, 3, R.Fast);

            R.wobbleAuto();
        }
    }
}
