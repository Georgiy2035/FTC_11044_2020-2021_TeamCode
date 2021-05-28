package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.InputStreamReader;

import static android.os.ParcelFileDescriptor.MODE_WORLD_READABLE;

@Autonomous(name = "AutoSet")
public class AutoSet extends LinearOpMode {

    Robot2020 R = new Robot2020();

    @Override
    public void runOpMode() throws InterruptedException {
        R.gamepadInit(gamepad1, gamepad2);
        R.initHWD(telemetry, hardwareMap, this);
        R.init();
        R.setNull();

        R.SH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //R.setNull();

        R.holdLift.start();
        //R.shootWrite.start();
        R.flagLift = false;
        waitForStart();
        while(!isStopRequested()){
            R.autoSet();
        }
    }
}
