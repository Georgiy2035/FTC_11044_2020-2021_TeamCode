package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

public class Robot2020Auto{

    double parstics(double sq) {
        double d = 0.059 / 0.6, tpr = 1024;
        return tpr * sq / (d * Math.PI);
    }

    double parssq(double tics) {
        double d = 0.059 / 0.6, tpr = 1024;
        return d * Math.PI * tics / tpr;
    }
    /**********************AUTONOMOUS_FUNCTIONS*****************************/

    public void GoTo(double x, double y, Robot2020 R, RunMode RM) {
        double p1 = 1, p2 = 1, p3 = 1, p4 = 1, px = 1, py = 1;
        double cor = 100, cor2 = R.RobotAngle - R.getAngle(), dcor = 0, correction = 0, minErAng = RM.minA;
        double Pex = 0, Pey = 0, Dex = 0, Dey = 0;
        double kPx = Const.KPx, kPy = Const.KPy, kA = Const.KA, kDx = Const.KDx, kDy = Const.KDy, v, tmp;

        Vector te = new Vector(parstics(x), parstics(y)),
                Er0 = te.dif(R.Rp).rotate(R.RobotAngle),
                Er = new Vector(Er0.X, Er0.Y),
                Er2 = new Vector(Er0.X, Er0.Y),
                r0 = new Vector(0, 0),
                r1 = new Vector(0, 0),
                r2 = new Vector(-R.LF.getCurrentPosition(), R.LB.getCurrentPosition()),
                dr,
                minEr = new Vector(RM.minX, RM.minY);

        te.X = Er0.X * sqrt(3) / Math.max(abs(Er0.X * sqrt(3)), abs(Er0.Y));
        te.Y = Er0.Y / Math.max(abs(Er0.X * sqrt(3)), abs(Er0.Y));

        R.tele.addData("X", te.X);
        R.tele.addData("Y", te.Y);
        R.tele.addData("RpX", R.Rp.X);
        R.tele.addData("RpY", R.Rp.Y);
        R.tele.addData("curAngle", R.getAngle());
        R.tele.addData("globalAngle", R.RobotAngle);
        R.tele.update();

        R.delay(0);

        /***LOOP_START***/

        while (!R.L.isStopRequested()
                && (abs(cor) > minErAng || abs(dcor) > 0.1 * Const.KAD
                || abs(Dey) > kDy * 3 || abs(Dex) > kDx * 3
                || abs(Er.X) > minEr.X || abs(Er.Y) > minEr.Y)) {

            v = R.voltageSensor.getVoltage();
            cor = R.RobotAngle - R.getAngle();

            r1.set(-R.LF.getCurrentPosition(), R.LB.getCurrentPosition());
            dr = r1.dif(r2);
            r2.set(r1.X, r1.Y);

            r0 = r0.sum(dr.rotate(cor));

            Er = Er0.dif(r0);

            Pex = Er.X * kPx; //Пропорциональная составляющая
            Pey = Er.Y * kPy;

            Dex = (Er.X - Er2.X) * kDx; //Дифференциальная составляющая
            Dey = (Er.Y - Er2.Y) * kDy;
            dcor = cor - cor2;

            Er2.set(Er.X, Er.Y);
            cor2 = cor;

            if (abs(Er.X) > Const.ticsx)
                px = te.X;
            else {
                if (Dex == 0 && abs(Er.X) > minEr.X)
                    kPx *= 1.2;
                else if (abs(Er.X) > 2 * minEr.X)
                    kPx = Const.KPx;
                else if (abs(Dex) > abs(Pex))
                    Dex = -Pex;

                px = Pex + Dex + signum(Er.X) * Const.minx;
            }

            if (abs(Er.Y) > Const.ticsy)
                py = te.Y * RM.speed;
            else {
                if (Dey == 0 && abs(Er.Y) > minEr.Y)
                    kPy *= 1.2;
                else if (abs(Er.Y) > 2 * minEr.Y)
                    kPy = Const.KPy;
                else if (abs(Dey) > abs(Pey))
                    Dey = -Pey;

                py = (Pey + Dey + signum(Er.Y) * Const.miny);
                py = Math.min(py, RM.speed);
            }

            if (dcor == 0 && abs(cor) > 0.5 && abs(Er.Y) + abs(Er.X) < 200)
                kA *= 1.1;
            else if (abs(cor) > minErAng && kA > Const.KA)
                kA = Const.KA;

            correction = cor * (abs(py) + abs(px) / 2 + Const.mincorr) * kA + dcor * Const.KAD; //Поправка на угол

            p1 = -(py + px - correction); //* 12 / v;
            p2 = -(py - px + correction); //* 12 / v;
            p3 = -(py + px + correction); //* 12 / v;
            p4 = -(py - px - correction); //* 12 / v;

            R.tele.addData("Erx", Er.X);
            R.tele.addData("Ery", Er.Y);
            R.tele.addData("rx", r0.X);
            R.tele.addData("ry", r0.Y);
            R.tele.addData("r1x", r1.X);
            R.tele.addData("r1y", r1.Y);
            R.tele.addData("Dex", Dex);
            R.tele.addData("Dey", Dey);
            R.tele.addData("p ", abs(p1) + abs(p2) + abs(p3) + abs(p4));
            R.tele.addData("cor", cor);
            R.tele.addData("dcor", dcor * Const.KAD);

            R.tele.update();

            R.motorsSetPower(p1, p2, p3, p4);
            R.Gr.Y = r0.Y;
        }

        /***LOOP_END***/

        R.Rp = R.Rp.sum(r0.rotate(-R.RobotAngle));

        R.motorsSetPower(0, 0, 0, 0);
    }

    public void turnTo(double TurnAngle, Robot2020 R) {
        double p1 = 1, p2 = 1, p3 = 1, p4 = 1;
        double Er = 1000, Er2 = 1000, kP = Const.KP, kD = Const.KD;
        double Pe = 0, Ie = 0, De = 0, v, v0 = 12, p;
        double minRegAng = 2;

        R.RobotAngle = TurnAngle;

        R.tele.addData("Mode", "TURN");
        R.tele.update();

        while (!R.L.isStopRequested() && (abs(De) > 0.01 * kD || abs(Er) > minRegAng)) {//0.4
            v = R.voltageSensor.getVoltage();
            Er = R.RobotAngle - R.getAngle();

            Pe = Er * kP;                 //Пропорциональная составляющая

            De = pow((Er - Er2), 3) * kD * pow((Const.ang / (Const.ang + abs(Er) * 99)), 1);         //Дифференциальная составляющая

            if (abs(Er) > Const.ang)
                p = signum(Er);
            else {
                if (De == 0 && abs(Er) > minRegAng)
                    kP *= 1.2;
                else if (abs(Er) > 2 * minRegAng)
                    kP = Const.KP;
                else if (abs(De) > abs(Pe))
                    De = -Pe;

                p = Pe + De + signum(Er) * Const.mincorra;
            }

            p1 = p; //* v0 / v;
            p2 = -p; //* v0 / v;
            p3 = -p; //* v0 / v;
            p4 = p; //* v0 / v;

            R.tele.addData("cor", Pe);
            R.tele.addData("dcor", De);
            R.tele.update();

            R.motorsSetPower(p1, p2, p3, p4);

            Er2 = Er;
        }
        R.motorsSetPower(0, 0, 0, 0);
    }

    public void turnToMin(double TurnAngle, Robot2020 R) {
        R.RobotAngle = TurnAngle;

        double p1 = 1, p2 = 1, p3 = 1, p4 = 1;
        double Er = 1000, Er2 = R.RobotAngle - R.getAngle();
        double Pe = 0, De = 0, kP = Const.KPm, kD = Const.KDm, v, v0 = 12;

        R.tele.addData("Mode", "TURN");
        R.tele.update();

        while (!R.L.isStopRequested() && (abs(De) > 0.0005 * kD || abs(Er) > 0.5)) {//0.4
            v = R.voltageSensor.getVoltage();
            Er = R.RobotAngle - R.getAngle();

            Pe = Er * kP;                 //Пропорциональная составляющая

            De = (Er - Er2) * kD;         //Дифференциальная составляющая

            if (De == 0 && abs(Er) > 0.5) {
                kP *= 1.2;
            }
            else if (abs(De) > abs(Pe))
                De = -Pe;
            if (Er * ErPrev < 0)
                kD *= 1.2;


            p1 = (Pe + De) * v0 / v;
            p2 = -(Pe + De) * v0 / v;
            p3 = -(Pe + De) * v0 / v;
            p4 = (Pe + De) * v0 / v;

            R.tele.addData("cor", Pe);
            R.tele.addData("dcor", De);
            R.tele.update();

            R.motorsSetPower(p1, p2, p3, p4);

            Er2 = Er;
        }
        R.motorsSetPower(0, 0, 0, 0);
    }

    double Er = 0, ErPrev = 0, p, d, i = 0; // угловая скорость шутера, считываемая хабом
    double shootPwr = 0, wobblePwr = 0, grabPwr = 0, liftPwr = 0;              // мощности, подаваемые на узлы
    double servoPullPos = 0, servoWobblePos = 0.7;                               // позиции сервомоторов
    boolean ShIsColibrate = true, liftIsDown = false;
    int fCatch = 0;

    void holdLift(Robot2020 R, FtcDashboard dashboard) {
        double v, v0 = 12;
        while (R.L.opModeIsActive() && !R.L.isStopRequested()) {
           R.L.resetStartTime();
            v = R.voltageSensor.getVoltage();
            if (R.flagLift) {
                R.UP.setPower(R.button.isPressed() ? 0.001 * v0 / v : 0.09 * v0 / v);  //(1 - UP.getCurrentPosition() / 100) * 0.4 + 0.005);
                R.RA.liftIsDown = false;
            }
            else if (!liftIsDown) {
                R.UP.setPower(-0.5);
                R.delay(350);
                R.UP.setPower(0);
                liftIsDown = true;
            }
            else
                R.UP.setPower(0);
            if (fCatch == 1)
                R.GR.setPower(1);
            else
                R.GR.setPower(0);

            if (R.sh == 1) {
                Er = Const.goalPosA - R.SH.getVelocity();
                if (ShIsColibrate && R.SH.getVelocity() < Const.goalPosA - 10) {
                    p = Er * Const.pidShStartA.p;
                    i += (Er + ErPrev) / 2 * Const.pidShStartA.i;
                    d = (Er - ErPrev) * Const.pidShStartA.d;
                }
                else {
                    p = Er * Const.pidShStableA.p;
                    i += (Er + ErPrev) / 2 * Const.pidShStableA.i;
                    d = (Er - ErPrev) * Const.pidShStableA.d;
                    ShIsColibrate = false;
                }
                shootPwr = p + d + i;
            }
            else if (R.sh == 2) {
                Er = Const.targetPosA - R.SH.getVelocity();
                if (ShIsColibrate && R.SH.getVelocity() < Const.targetPosA - 10) {
                    p = Er * Const.pidShStartAt.p;
                    i += (Er + ErPrev) / 2 * Const.pidShStartAt.i;
                    d = (Er - ErPrev) * Const.pidShStartAt.d;
                }
                else {
                    p = Er * Const.pidShStableAt.p;
                    i += (Er + ErPrev) / 2 * Const.pidShStableAt.i;
                    d = (Er - ErPrev) * Const.pidShStableAt.d;
                    ShIsColibrate = false;
                }
                shootPwr = p + d + i;
            }
            else if (R.sh == 0) {
                ShIsColibrate = true;
                shootPwr = 0;
                i = 0;
            }
            R.ShPower(shootPwr);
            /* мощность, подаваемая на шутер */

            /* присваивание старому значению ошибки новое */

            ErPrev = Er;

            /* вывод показаний в FTC Dashboard */

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("speed", R.SH.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            R.L.getRuntime();
        }
    }

    public void wobbleAuto(Robot2020 R)
    {
        double v, v0 = 12;
        if(!R.L.isStopRequested()) {
            v = R.voltageSensor.getVoltage();
            R.VB.setPower(0.6 * v0 / v);
            R.delay(700);
            R.VB.setPower(0);
            R.delay(100);
            R.servoWobble.setPosition(0);
            R.delay(500);
            R.VB.setPower(-0.6 * v0 / v);
            R.delay(500);
            R.VB.setPower(0);
        }
    }

    void pushRing (Robot2020 R){
        if(!R.L.isStopRequested()) {
            R.pullPos(0.21);
            R.delay(180);
            R.pullPos(0.73);
            R.delay(180);
        }
    }

/********************AUTO_SET*************************/

    boolean aWasDown = false, bWasDown = false, aWasDown2 = false, bWasDown2 = false, xWasDown2 = false, yWasDown2 = false, lbWasDown2 = false, rbWasDown2 = false;

    void autoSet (Robot2020 R) {
        if(R.gamepad1.x && !R.xWasDown) {
            Const.KPy += 0.0001;
            R.xWasDown = true;
        }
        else if(R.gamepad1.y && !R.yWasDown) {
            Const.KPy -= 0.0001;
            R.yWasDown = true;
        }
        else if(R.gamepad1.dpad_left && !R.leftWasDown) {
            Const.KDy -= 0.0005;
            R.leftWasDown = true;
        }
        else if(R.gamepad1.dpad_right && !R.rightWasDown) {
            Const.KDy += 0.0005;
            R.rightWasDown = true;
        }
        else if (R.gamepad1.dpad_up && !R.dpadUpWasDown) {
            Const.KPx += 0.0001;
            R.dpadUpWasDown = true;
        }
        else if (R.gamepad1.dpad_down && !R.dpadDownWasDown) {
            Const.KPx -= 0.0001;
            R.dpadDownWasDown = true;
        }
        else if (R.gamepad1.left_bumper && !R.leftBumperWasDown) {
            Const.KDx -= 0.05;
            R.leftBumperWasDown = true;
        }
        else if (R.gamepad1.right_bumper && !R.rightBumperWasDown) {
            Const.KDx += 0.05;
            R.rightBumperWasDown = true;
        }
        else if (R.gamepad1.a && !aWasDown) {
            Const.miny += 0.01;
            aWasDown = true;
        }
        else if (R.gamepad1.b && !bWasDown) {
            Const.miny -= 0.01;
            bWasDown = true;
        }
        else if (R.gamepad2.a && !aWasDown2) {
            Const.mincorr += 0.01;
            aWasDown2 = true;
        }
        else if (R.gamepad2.b && !bWasDown2) {
            Const.mincorr -= 0.01;
            bWasDown2 = true;
        }
        else if (R.gamepad2.x && !xWasDown2) {
            Const.KA += 0.01;
            xWasDown2 = true;
        }
        else if (R.gamepad2.y && !yWasDown2) {
            Const.KA -= 0.01;
            yWasDown2 = true;
        }
        else if (R.gamepad2.right_bumper && !rbWasDown2) {
            Const.KAD += 0.001;
            rbWasDown2 = true;
        }
        else if (R.gamepad2.left_bumper && !lbWasDown2) {
            Const.KAD -= 0.001;
            lbWasDown2 = true;
        }

        if (R.xWasDown == true && !R.gamepad1.x)
            R.xWasDown = false;
        if (R.yWasDown == true && !R.gamepad1.y)
            R.yWasDown = false;
        if (R.dpadUpWasDown == true && !R.gamepad1.dpad_up)
            R.dpadUpWasDown = false;
        if (R.dpadDownWasDown == true && !R.gamepad1.dpad_down)
            R.dpadDownWasDown = false;
        if (R.leftBumperWasDown == true && !R.gamepad1.left_bumper)
            R.leftBumperWasDown = false;
        if (R.rightBumperWasDown == true && !R.gamepad1.right_bumper)
            R.rightBumperWasDown = false;
        if (R.xWasDown == true && !R.gamepad1.x)
            R.xWasDown = false;
        if (R.yWasDown == true && !R.gamepad1.y)
            R.yWasDown = false;
        if (R.leftBumperWasDown == true && !R.gamepad1.left_bumper)
            R.leftBumperWasDown = false;
        if (R.rightBumperWasDown == true && !R.gamepad1.right_bumper)
            R.rightBumperWasDown = false;
        if (R.dpadUpWasDown == true && !R.gamepad1.dpad_up)
            R.dpadUpWasDown = false;
        if (R.dpadDownWasDown == true && !R.gamepad1.dpad_down)
            R.dpadDownWasDown = false;
        if (R.leftWasDown == true && !R.gamepad1.dpad_left)
            R.leftWasDown = false;
        if (R.rightWasDown == true && !R.gamepad1.dpad_right)
            R.rightWasDown = false;
        if (aWasDown == true && !R.gamepad1.a)
            aWasDown = false;
        if (bWasDown == true && !R.gamepad1.b)
            bWasDown = false;
        if (aWasDown2 == true && !R.gamepad2.a)
            aWasDown2 = false;
        if (bWasDown2 == true && !R.gamepad2.b)
            bWasDown2 = false;
        if (xWasDown2 == true && !R.gamepad2.x)
            xWasDown2 = false;
        if (yWasDown2 == true && !R.gamepad2.y)
            yWasDown2 = false;
        if (rbWasDown2 == true && !R.gamepad2.right_bumper)
            rbWasDown2 = false;
        if (lbWasDown2 == true && !R.gamepad2.left_bumper)
            lbWasDown2 = false;

        if (R.gamepad1.right_stick_button)
            R.GoTo(0.75, 2.5, R.Fast);
        if (R.gamepad1.left_stick_button)
            R.GoTo(-1, 2, R.Fast);
        if (R.gamepad2.right_stick_button) {
            R.GoTo(0, 1, R.Fast);
            R.turnTo(45);
            R.GoTo(1, 2, R.Fast);
        }
        if (R.gamepad2.left_stick_button)
            R.GoTo(1, 1.5, R.Fast);

        R.tele.addData("KPy", Const.KPy);
        R.tele.addData("KDy", Const.KDy);
        R.tele.addData("KPx", Const.KPx);
        R.tele.addData("KDx", Const.KDx);
        R.tele.addData("miny", Const.miny);
        R.tele.addData("mincorr", Const.mincorr);
        R.tele.addData("KA", Const.KA);
        R.tele.addData("KAD", Const.KAD);

        R.tele.update();
    }

    void autoSetTurnTo (Robot2020 R) {
        if(R.gamepad1.x && !R.xWasDown) {
            Const.KPm += 0.001;
            R.xWasDown = true;
        }
        else if(R.gamepad1.y && !R.yWasDown) {
            Const.KPm -= 0.001;
            R.yWasDown = true;
        }
        else if(R.gamepad1.dpad_left && !R.leftWasDown) {
            Const.KDm -= 0.001;
            R.leftWasDown = true;
        }
        else if(R.gamepad1.dpad_right && !R.rightWasDown) {
            Const.KDm += 0.001;
            R.rightWasDown = true;
        }
        else if(R.gamepad1.a && !aWasDown) {
            Const.KP += 0.001;
            aWasDown = true;
        }
        else if(R.gamepad1.b && !bWasDown) {
            Const.KP -= 0.001;
            bWasDown = true;
        }
        else if(R.gamepad1.dpad_up && !R.dpadUpWasDown) {
            Const.KD += 0.001;
            R.dpadUpWasDown = true;
        }
        else if(R.gamepad1.dpad_down && !R.dpadDownWasDown) {
            Const.KD -= 0.001;
            R.dpadDownWasDown = true;
        }
        else if (R.gamepad1.dpad_up && !R.dpadUpWasDown) {
            //mincorra += 0.01;
            R.dpadUpWasDown = true;
        }
        else if (R.gamepad1.dpad_down && !R.dpadDownWasDown) {
            //mincorra -= 0.01;
            R.dpadDownWasDown = true;
        }

        if (R.xWasDown == true && !R.gamepad1.x)
            R.xWasDown = false;
        if (R.yWasDown == true && !R.gamepad1.y)
            R.yWasDown = false;
        if (R.dpadUpWasDown == true && !R.gamepad1.dpad_up)
            R.dpadUpWasDown = false;
        if (R.dpadDownWasDown == true && !R.gamepad1.dpad_down)
            R.dpadDownWasDown = false;
        if (R.leftWasDown == true && !R.gamepad1.dpad_left)
            R.leftWasDown = false;
        if (R.rightWasDown == true && !R.gamepad1.dpad_right)
            R.rightWasDown = false;
        if (aWasDown == true && !R.gamepad1.a)
            aWasDown = false;
        if (bWasDown == true && !R.gamepad1.b)
            bWasDown = false;
        if (R.dpadUpWasDown == true && !R.gamepad1.dpad_up)
            R.dpadUpWasDown = false;
        if (R.dpadDownWasDown == true && !R.gamepad1.dpad_down)
            R.dpadDownWasDown = false;
        if (R.gamepad1.right_stick_button)
            R.turnTo(60);
        if (R.gamepad1.left_stick_button)
            R.turnTo(-20);
        if (R.gamepad2.left_stick_button)
            R.turnToMin(6.22);
        if (R.gamepad2.right_stick_button)
            R.turnToMin(0);

        R.tele.addData("KP", Const.KP);
        R.tele.addData("KD", Const.KD);
        R.tele.addData("KPm", Const.KPm);
        R.tele.addData("KDm", Const.KDm);

        R.tele.update();
    }

    void autoReleSet(Robot2020 R){
        double px = 0.01, py = 0.01, pa = 0.01, D = 0, tmp = R.LF.getCurrentPosition();
        while (D == 0){
            D = R.LF.getCurrentPosition() - tmp;
            tmp = R.LF.getCurrentPosition();
            px *= 1.2;
            R.motorsSetPower(-px, -px, -px, -px);
        }
        R.motorsSetPower(0, 0, 0, 0);
        R.delay(2000);
        D = 0;
        tmp = R.LB.getCurrentPosition();
        while (D == 0){
            D = R.LB.getCurrentPosition() - tmp;
            tmp = R.LB.getCurrentPosition();
            py *= 1.2;
            R.motorsSetPower(-py, py, -py, py);
        }
        R.motorsSetPower(0, 0, 0, 0);
        R.delay(2000);
        D = 0;
        tmp = R.getAngle();
        while (D == 0){
            D = R.getAngle() - tmp;
            tmp = R.getAngle();
            pa *= 1.2;
            R.motorsSetPower(-py, py, py, -py);
        }
        R.motorsSetPower(0, 0, 0, 0);

        R.tele.addData("KX", px);
        R.tele.addData("KY", py);
        R.tele.addData("KA", pa);
        R.tele.update();
        R.delay(10000);
    }
    /***************************VUFORIA*********************************/

    public String PIC = "null";

    public static final String TAG = "Vuforia VuMark Sample";

    private VuforiaLocalizerImpl vuforia;

    public void VuforiaInit(Robot2020 R) {
        int cameraMonitorViewId = R.hwMp.appContext.getResources().getIdentifier(/*"cameraMonitorViewId"*/"Camera", "id", R.hwMp.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AaeuxIX/////AAAAGWkhcBRF1krXujWCTQksCL1qYMjajjzr8Wj1CxU6AcNrVHJXnj1Zm9Es+l+y95FgzCwUzVAZQtJyUfxvWum2uRameXDyH8aREWSdfFB3uSkXdS0VvRdYnX65Uj0IXOb6BgRBacBHxGSbcRfcGzNPgEBks05Zq6TpQYqZnln78IvyXzRxTPy1K87FGYqlq/4iV9aWuFkPuwcOUJXOXmMZmdQ2yBVCniyRUSFFuP7WjUMrSev0jhCE6ZqADuP9qmYmbiAP/COYXhlgmnZkbbQR1xRRD3GW7Qlq1TLxsfY15e7oK8KTxRhlj4CIVpGrI4aV3vYLQDCkhawckuyNi5YZZCbLAVUw1vwPd5vF5erLjCc2";
        vuforia = new VuforiaLocalizerImpl(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        CameraDevice.getInstance().setFlashTorchMode(true);
    }

    public Bitmap getImage() throws InterruptedException {
        Image img;
        // get current frame and transform it to Bitmap
        img = getImagefromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        Bitmap bm_img = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm_img.copyPixelsFromBuffer(img.getPixels());

        return bm_img;
    }

    @Nullable
    private Image getImagefromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }

        return null;
    }

    public int colorAnalyser(Bitmap bm_img, Robot2020 R) {
        int Y_COUNTER = 0; // color counters
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];

        int width = bm_img.getWidth(); // width in landscape mode
        int height = bm_img.getHeight(); // height in landscape mode

        CameraDevice.getInstance().setFlashTorchMode(false);

        for (int i = 0, di = 8; i < height; i += di)
            for (int j = 0, dj; j < width; j += dj) {
                cur_color_int = bm_img.getPixel(j, i);
                rgb[0] = Color.red(cur_color_int);
                rgb[1] = Color.green(cur_color_int);
                rgb[2] = Color.blue(cur_color_int); // 203 103 3

                if (Conus(215, 132, 29, 0.18, rgb[0], rgb[1], rgb[2]) == 1) {
                    Y_COUNTER++;//252 90 3
                    di = 2;
                    dj = 2;
                }
                else {
                    di = 8;
                    dj = 8;
                }
            }

        int pos;
        if (Y_COUNTER < 75)
            pos = 1;
        else if (Y_COUNTER < 350)
            pos = 2;
        else
            pos = 3;

        R.tele.addData("POSITION", pos);
        R.tele.addData("Y_COUNTER:", Y_COUNTER);
        R.tele.update();
        R.delay(000);
        return pos;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public int Conus( double x1, double y1, double z1, double alpha, double r, double g, double b) {
        if (alpha > Math.acos((x1 * r + y1 * g + z1 * b) / (sqrt(x1 * x1 + y1 * y1 + z1 * z1) * sqrt(r * r + g * g + b * b))))
            return 1;
        return 0;
    }
}

