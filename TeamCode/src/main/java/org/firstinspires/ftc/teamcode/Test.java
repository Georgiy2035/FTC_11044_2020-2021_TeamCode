package org.firstinspires.ftc.teamcode;

import android.os.Environment;

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

@TeleOp(name = "Test")
public class Test extends LinearOpMode {

    Robot2020 R = new Robot2020();

    @Override
    public void runOpMode() throws InterruptedException {
        R.gamepadInit(gamepad1, gamepad2);
        R.initHWD(telemetry, hardwareMap, this);
        R.init();

        R.SH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        R.setNull();
 //       R.shootWrite.start();
        waitForStart();
        //while(!isStopRequested()){
            //R.UP.setPower(R.button.isPressed() ? 0.0001 : 0.4);
            //R.GoTo(0, 1);
       // }

        try {
            File f = new File(Environment.getExternalStorageDirectory().toString() + "/" + "ShootTable");
            f.createNewFile();                                         // Создается файл, если он не был создан
            FileOutputStream outputStream = new FileOutputStream(f);   // После чего создаем поток для записи
            for(int j = 0; j < R.i; j++) {
                outputStream.write((Double.toString(R.time[j]) + " "  + Double.toString(R.speedVel[j]) + " " + Double.toString(R.volt[j]) + "00" + "\n").getBytes());                            // и производим непосредственно запись
            }
            outputStream.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

    }
}
