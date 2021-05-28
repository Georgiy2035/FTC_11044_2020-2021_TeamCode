package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    Telemetry tele;
    HardwareMap hwMp;
    FtcDashboard dashboard;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;

    public abstract void initHWD(Telemetry tele, HardwareMap hwMp, LinearOpMode L);
    public abstract void init();

    public void initDashboard(FtcDashboard dashboard)
    {
        this.dashboard = dashboard;
    }

    void gamepadInit( Gamepad gamepad1, Gamepad gamepad2 )
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public final void delay(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}