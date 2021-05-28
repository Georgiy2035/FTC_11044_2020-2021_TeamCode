package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.BufferedWriter;
import java.io.FileOutputStream;

@Autonomous(name = "AutoTest")
public class TestAuto extends LinearOpMode {
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

        telemetry.addData("Mode", "running");
        telemetry.update();

        Bitmap bm = R.getImage();
        position = R.colorAnalyser(bm);

        telemetry.addData("Position", position);
        telemetry.update();

        R.delay(1000);

        R.holdLift.start();
        R.flagLift = true;
        R.sh = 2;

        R.delay(1700);

        R.pushRing();
        R.delay(00);
        R.pushRing();
        R.delay(000);
        R.pushRing();

        while (!isStopRequested()){
            R.L.resetStartTime();

            telemetry.addData("vel", R.SH.getVelocity());

            telemetry.addData("p", R.RA.p);

            telemetry.addData("i", R.RA.i);

            telemetry.addData("d", R.RA.d);
            telemetry.update();

            /*TelemetryPacket packet = new TelemetryPacket();
            packet.put("speedA", R.SH.getVelocity());
            R.dashboard.sendTelemetryPacket(packet);
*/
            R.L.getRuntime();
        }
        /*R.motorsSetPower(-1,-1, -1, -1);
        R.delay(1000);
        R.motorsSetPower(0,0, 0, 0);
        while (!isStopRequested()) {
            telemetry.addData("LF", R.LF.getCurrentPosition());
            telemetry.addData("RF", R.LB.getCurrentPosition());
            telemetry.update();
        }*/



/*        R.holdLift.start();

        R.delay(1000);

        R.SH.setPower(1);

        R.delay(500);

        for (int i = 0; i < 4 && !isStopRequested(); i++) {
            R.servoPull.setPosition(0.4);
            R.delay(500);
            R.servoPull.setPosition(0);
            R.delay(1000);
        }

        R.flag = false;

        while (R.UP.getCurrentPosition() > 15) {
            R.UP.setPower(-0.4);
        }
*/
/*
        R.servoPull.setPosition(0.5);
        R.delay(250);
        R.servoPull.setPosition(0);
        R.delay(1000);

        R.holdLift2.stop();
        R.holdLift3.start();

        for (int i = 0; i < 2; i++) {
            R.servoPull.setPosition(0.5);
            R.delay(250);
            R.servoPull.setPosition(0);
            R.delay(1000);
        }*/

     /*   R.SH.setPower(0);
        R.delay(1000);

        R.GoTo(0, 0.5);
    */
        try {
            File f = new File(Environment.getExternalStorageDirectory().toString() + "/" + "ShootTable");
            f.createNewFile();                                         // Создается файл, если он не был создан
            FileOutputStream outputStream = new FileOutputStream(f);   // После чего создаем поток для записи
            for (int j = 0; j < R.i; j++) {
                outputStream.write((Double.toString(R.time[j]) + " " + Double.toString(R.speedVel[j]) + " " + Double.toString(R.volt[j]) + "00" + "\n").getBytes());                            // и производим непосредственно запись
            }
            outputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

