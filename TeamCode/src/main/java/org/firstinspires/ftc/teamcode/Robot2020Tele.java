package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class Robot2020Tele {

    /*****************************CONSTANTS*********************************/

    double v, v0 = 12;                                                         // напряжение на аккумуляторе, считываемое хабом
    double oldtime;                                                            // пройденное время
    double shootPwr = 0, wobblePwr = 0, grabPwr = 0, liftPwr = 0;              // мощности, подаваемые на узлы
    double servoPullPos = 0, servoWobblePos = 0.7;                             // позиции сервомоторов

    /**********************WB CONTROL & SHOOT CONTROL***************************/

    public void TOmniWB(Robot2020 R, FtcDashboard dashboard) {
        while (!R.L.isStopRequested()) {
            R.L.resetStartTime();
            /* объявление переменных */

            //R.i++;                                    // новая ячейка в массиве
            R.masx[R.i] = R.LB.getCurrentPosition();    // запись в массив показаний с энкодера
            R.masy[R.i] = R.LF.getCurrentPosition();    // запись в массив показаний с энкодера
            R.masang[R.i] = -R.getAngle();              // запись в массив показаний с хаба

            double x = R.gamepad1.left_stick_x + (R.gamepad1.dpad_right ? 0.55 : 0) + (R.gamepad1.dpad_left ? -0.55 : 0);
            double y = -(R.gamepad1.left_stick_y + (R.gamepad1.dpad_up ? -0.45 : 0) + (R.gamepad1.dpad_down ? 0.45 : 0));
            double lt = -(pow(R.gamepad1.left_trigger, 3) + (R.gamepad1.right_bumper ? -0.3 : 0));
            double rt = -(pow(R.gamepad1.right_trigger, 3) + (R.gamepad1.left_bumper ? -0.3 : 0));
            double l = x * x + y * y;

            double x1 = x * 1 + y * sqrt(3);
            double y1 = -x * 1 + y * sqrt(3);

            if (abs(x1) >= abs(y1) && x1 != 0) {
                y1 /= abs(x1);
                x1 /= abs(x1);
            } else if (abs(y1) >= abs(x1) && y1 != 0) {
                x1 /= abs(y1);
                y1 /= abs(y1);
            }

            x1 *= l;
            y1 *= l;

            /* запуск моторов */

            ShootRun(R);
            WbRun(R);

            /* вывод показаний в FTC Dashboard */

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("speed", R.SH.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            /* подача напряжений на моторы и сервомоторы */

            R.ShPower(shootPwr);    // мощность подаваемая на шутер
            R.WbPower(wobblePwr);   // мощность для стабильного захватывания воббла
            R.motorsSetPower((-x1 - lt + rt), (-y1 + lt - rt), (-x1 + lt - rt), (-y1 - lt + rt)); // мощности, подаваемые на колеса

            R.L.getRuntime();
        }
    }

    /**********************************TELEOP_FUNCTION**********************************/

    /* объявление переменных для управляемого периода */

    public void teleMove( Robot2020 R )        // телеуправляемый алгоритм для всех узлов
    {
        v = R.voltageSensor.getVoltage();      // считывание напряжения в данный момент времени

        /* проверка нажатий кнопок с геймпада */

        if (R.gamepad1.a && upFlag) {
            R.pullPos(0.20);
            R.delay(165);
            R.pullPos(0.73);
            R.delay(165);
        }
        else if (R.gamepad1.b && !rightWasDown && upFlag) {
            R.pullPos(0.20);
            R.delay(165);
            R.pullPos(0.73);
            R.delay(165);
            R.pullPos(0.20);
            R.delay(165);
            R.pullPos(0.73);
            R.delay(165);
            R.pullPos(0.20);
            R.delay(165);
            R.pullPos(0.73);
            R.delay(165);
        }

        if (R.gamepad2.b) {
            upFlag = false;
            liftPwr = -0.3 * 12 / v;
            R.UpPower(liftPwr);   // мощность подаваемая на лифт
            R.delay(300);
            liftPwr = 0;
        }
        else if (R.gamepad2.a || upFlag)
        {
            upFlag = true;
            liftPwr = R.button.isPressed() ? 0.165 * 12 / v : 0.38 * 12 / v;
        }
        else if (R.gamepad2.x)
            R.servoWobblePos = 0.7;
        else if (R.gamepad2.y)
            R.servoWobblePos = 0.2;

        /* подача мощностей на моторы и сервомоторы */

        grabPwr = !R.button.isPressed() ?  R.gamepad2.left_stick_y * 1 * 12 / v : 0;

        R.GrPower(grabPwr);   // мощность для стабильного захватывания кольца
        R.UpPower(liftPwr);   // мощность подаваемая на лифт

        R.wobblePos(R.servoWobblePos); // мощность подаваемая на серво-захват воббла

        /* вывод значений на DS */

        R.tele.addData("SH", shootPwr);
        R.tele.addData("SHTics", R.SH.getVelocity());
        R.tele.addData("Error ", Er);
        R.tele.addData("SPush ", R.servoPull.getPosition());

        R.tele.addData("p ", p);
        R.tele.addData("d ", d);
        R.tele.addData("i ", i);

        R.tele.addData("Voltage ", v);
        R.tele.addData("Button", R.button.isPressed() ? "Pressed" : "Not pressed");

        /*
        R.tele.addData("UP", liftPwr);
        R.tele.addData("UPTics", R.UP.getCurrentPosition());
        R.tele.addData("Error ", ErUp);
        R.tele.addData("pUp ", pUp);

        R.tele.addData("WB", wobblePwr);
        R.tele.addData("WBTics", R.VB.getCurrentPosition());
        R.tele.addData("Error ", ErWb);
        R.tele.addData("pWb ", pWb);
        */

        R.tele.update();
    }

    boolean upWasDown = false, rightWasDown = false;
    boolean ShIsNotColibrate = true;
    double Er = 0, ErPrev = 0;
    double p, d, i = 0;

    public void ShootRun( Robot2020 R )         // пид регулятор для мотора шутера
    {
        /* проверка нажатий с геймпада */

        if (R.gamepad2.dpad_down) {
            upWasDown = false;
            rightWasDown = false;
            ShIsNotColibrate = true;
            shootPwr = 0;
            i = 0;
        }
        else if (R.gamepad2.dpad_up || upWasDown) {
            upWasDown = true;
            rightWasDown = false;
            Er = Const.goalPos - R.SH.getVelocity();
            if (ShIsNotColibrate && R.SH.getVelocity() < Const.goalPos - 10) {
                p = Er * Const.pidShStart.p;
                i += (Er + ErPrev) / 2 * Const.pidShStart.i;
                d = (Er - ErPrev) * Const.pidShStart.d;
            }
            else if (ShIsNotColibrate) {
                i = 0;
                ShIsNotColibrate = false;
            }
            else {
                p = Er * Const.pidShStable.p;
                i += (Er + ErPrev) / 2 * Const.pidShStable.i;
                d = (Er - ErPrev) * Const.pidShStable.d;
            }
            /* мощность, подаваемая на шутер */

            shootPwr += p + d + i;
            //shootPwr = p + d + i;
        }
        else if (R.gamepad2.dpad_right || rightWasDown) {
            rightWasDown = true;
            upWasDown = false;
            Er = Const.targetPos - R.SH.getVelocity();
            if (ShIsNotColibrate && R.SH.getVelocity() < Const.targetPos - 10) {
                p = Er * Const.pidShStart.p;
                i += (Er + ErPrev) / 2 * Const.pidShStart.i;
                d = (Er - ErPrev) * Const.pidShStart.d;
            }
            else if (ShIsNotColibrate) {
                i = 0;
                ShIsNotColibrate = false;
            }
            else {
                p = Er * Const.pidShStable.p;
                i += (Er + ErPrev) / 2 * Const.pidShStable.i;
                d = (Er - ErPrev) * Const.pidShStable.d;
            }
            /* мощность, подаваемая на шутер */

            shootPwr += p + d + i;
            //shootPwr = p + d + i;
        }

        /* присваивание старому значению ошибки новое */

        ErPrev = Er;

    }

    boolean upIsNotCalibrate = false;
    boolean upFlag = false, upNotFlag = false;
    double ErUp = 0, ErUpPrev = 0;
    double pUp, dUp, iUp = 0;

    public void UpRun( Robot2020 R )
    {
        if (R.gamepad2.b || upNotFlag)
        {
            upFlag = false;
            upNotFlag = true;
            if (upIsNotCalibrate && R.UP.getCurrentPosition() > 100) {
                ErUp = Const.upPos - R.UP.getCurrentPosition();
                pUp = ErUp * Const.pidUp.p;
                iUp += (Er + ErUpPrev) / 2 * Const.pidUp.i;
                dUp = (ErUp - ErUpPrev) * Const.pidUp.d;
                liftPwr = pUp + iUp + dUp;
            }
            else
            {
                upIsNotCalibrate = false;
                liftPwr = 0;
                R.UP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                R.UP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        if (R.gamepad2.a || upFlag)
        {
            upFlag = true;
            upNotFlag = false;
            upIsNotCalibrate = true;
            liftPwr = R.button.isPressed() ? 0.165 * 12 / v : 0.36 * 12 / v;
        }

        /* присваивание старому значению ошибки новое */

        ErUpPrev = ErUp;
    }

    double ErWb = 0;
    double pWb;

    public void WbRun( Robot2020 R )
    {
        if (R.gamepad2.right_trigger != 0)
        {
            ErWb = Const.wbPos1 - R.VB.getCurrentPosition();
            pWb = ErWb * Const.pidWb1.p;
            wobblePwr = pWb * R.gamepad2.right_trigger;
        }
        else if (R.gamepad2.left_trigger != 0)
        {
            ErWb = Const.wbPos2 - R.VB.getCurrentPosition();
            pWb = ErWb * Const.pidWb2.p;
            wobblePwr = pWb * R.gamepad2.left_trigger;
        }
        else
            wobblePwr = 0;
    }

    public void ServoPullRun( Robot2020 R ) {
        /*if (R.gamepad1.a && upFlag) {
            R.pullPos(0.8);
            R.delay(350);
            R.pullPos(0.18);
            R.delay(350);
        } else if (R.gamepad1.b && !rightWasDown && upFlag) {
            R.pullPos(0.8);
            R.delay(350);
            R.pullPos(0.18);
            R.delay(350);
            R.pullPos(0.8);
            R.delay(350);
            R.pullPos(0.18);
            R.delay(350);
            R.pullPos(0.8);
            R.delay(350);
            R.pullPos(0.18);
            R.delay(350);
        }*/
        if (R.gamepad1.a /*&& R.button.isPressed()*/)
        {
            if (R.servoPull.getPosition() <= 0.5)
                R.pullPos(0.5);
            else if (R.servoPull.getPosition() >= 0.18)
                R.pullPos(0.18);
        }
        else if (R.gamepad2.a /*&& R.button.isPressed() && !rightWasDown*/)
        {

        }
    }

    public void ShootGraphics( Robot2020 R, FtcDashboard dashboard )
    {
        //while (!R.L.isStopRequested()) {
            R.L.resetStartTime();

            Er = Const.goalPos - R.SH.getVelocity();
            p = Er * Const.pidShStable.p;
            i += (Er + ErPrev) / 2 * Const.pidShStable.i;
            d = (Er - ErPrev) * Const.pidShStable.d;
            ShIsNotColibrate = false;

            /* мощность, подаваемая на шутер */

            shootPwr += p + d + i;

            /* подача напряжения на шутер */

            R.ShPower(R.shootPwr);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("speed", R.SH.getVelocity());
            packet.put("button", R.button.isPressed());
            dashboard.sendTelemetryPacket(packet);

            /* присваивание старому значению скорости новое */

            ErPrev = Er;
            R.L.getRuntime();
       // }
    }

    /***************************GET_CORDS_NOW***********************************/

    void getCords( Robot2020 R )
    {
        double xi, yi, ai;
        R.X = 0;
        R.Y = 0;

        for (int j = 1; j < R.i; j++) {
            xi = R.masx[j] - R.masx[j - 1];
            yi = R.masy[j] - R.masy[j - 1];
            ai = (R.masang[j] + R.masang[j - 1]) * Math.PI / 360;
            R.X += xi * cos(ai) + yi * sin(ai);
            R.Y += yi * cos(ai) - xi * sin(ai);
        }
    }
}