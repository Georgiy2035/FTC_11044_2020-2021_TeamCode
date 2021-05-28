package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    Robot2020 R = new Robot2020();

    @Override
    public void runOpMode() throws InterruptedException {
        R.gamepadInit(gamepad1, gamepad2);
        R.initHWD(telemetry, hardwareMap, this);
        R.initDashboard(FtcDashboard.getInstance());
        R.init();

        R.SH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.UP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.VB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R.SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.UP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.VB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        R.TOmniWB.start();
        while(!isStopRequested()) {
            R.teleMove();
        }
    }
}
