package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TestEnc")
public class TestEnc extends LinearOpMode {

    Robot2020 R = new Robot2020();


    @Override
    public void runOpMode() throws InterruptedException {
        R.gamepadInit(gamepad1, gamepad2);
        R.initHWD(telemetry, hardwareMap, this);
        R.init();

        R.LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        R.RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        R.UP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        R.VB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        R.LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        R.RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.VB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R.VB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(!isStopRequested())
            R.testEnc();
    }
}