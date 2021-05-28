 package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;
import java.io.FileOutputStream;

 public class Robot2020 extends Robot {

    /*********************ROBOT_FIELDS**************************/

    Robot2020Auto RA = new Robot2020Auto();
    Robot2020Tele RT = new Robot2020Tele();

    DcMotor LF, RF, RB, LB;
    DcMotorEx SH;
    DcMotor UP, GR, VB;
    Servo servoPull, servoWobble;
    ModernRoboticsTouchSensor button;
    double X = 0, Y = 0;
    int sh = 0;
    Vector Rp = new Vector(0, 0),
     Gr = new Vector(0, 0);

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    VoltageSensor voltageSensor;

    boolean flagLift = true;

    double speedShv, deltaSpeedShv = 0, averageSpeedShv = 0; // угловая скорость шутера, считываемая хабом
    double shootPwr = 0;              // мощности, подаваемые на узлы
    double servoWobblePos = 0.7, v;                               // позиции сервомоторов, напряжение на аккумуляторе, считываемое хабом
    boolean rightWasDown = false, leftWasDown = false, yWasDown = false, xWasDown = false;
    boolean leftBumperWasDown = false, rightBumperWasDown = false, dpadUpWasDown = false, dpadDownWasDown = false;
    int i = 0;
    double oldtime;
    double[] speedVel = new double[10000], deltaSpeedVel = new double[10000], averageSpeedVel = new double[10000], volt = new double[10000], time = new double[10000]; // массив скорости шутера
    double[] masx = new double[10000], masy = new double[10000], masang = new double[10000];

    RunMode Fast = new RunMode(1, 200,200, 3),
            Accuracy = new RunMode(1, 60, 80, 1),
            Slow = new RunMode(0.2, 60, 150, 3);

    /**********************CONSTANTS****************************/

    double globalAngle = 0, StartAngle = 0, RobotAngle = 0;
    double slow = 1;

    /********************INIT_FUNCTIONS*************************/
    @Override
    public void initHWD(Telemetry tele, HardwareMap hwMp, LinearOpMode L) {
        this.hwMp = hwMp;
        this.tele = tele;
        this.L = L;
    }

    @Override
    public void init() {
        imu = hwMp.get(BNO055IMU.class, "gyro");

        RF = hwMp.get(DcMotor.class, "FrontRightDc");
        LF = hwMp.get(DcMotor.class, "FrontLeftDc");
        RB = hwMp.get(DcMotor.class, "BackRightDc");
        LB = hwMp.get(DcMotor.class, "BackLeftDc");

        GR = hwMp.get(DcMotor.class, "GrabDc");
        SH = hwMp.get(DcMotorEx.class, "ShootDc");
        UP = hwMp.get(DcMotor.class, "LiftDc");
        VB = hwMp.get(DcMotor.class, "BlkRotateDc");

        servoPull = hwMp.get(Servo.class, "ServoPull");
        servoWobble = hwMp.get(Servo.class, "WobbleGrab");

        button = hwMp.get(ModernRoboticsTouchSensor.class, "button");

        voltageSensor = hwMp.voltageSensor.iterator().next();

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        UP.setDirection(DcMotorSimple.Direction.REVERSE);
        VB.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        UP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        UP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**************IMU**************/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            delay(30);
            tele.addData("Wait", "Calibrating");
            tele.update();
        }
        resetAngle();
        RobotAngle = 0;//getAngle();
        tele.addData("Ok", "Calibrated");
        tele.update();
    }
/**********************END_OF_INIT**************************/

    /****************SUPPORT_FUNCTIONS******************/

    public void diagonal() {
        motorsSetPower(gamepad1.left_stick_x, 0, gamepad1.left_stick_x, 0);
    }

    /****************SUPPORT_MOTORS_FUNCTIONS******************/

    public void motorsSetPower( double lf, double rf, double rb, double lb ) {
        LF.setPower(lf);
        RF.setPower(rf);
        RB.setPower(rb);
        LB.setPower(lb);
    }

    public void ShPower( double Sh ) {
        SH.setPower(Sh);
    }

    public void GrPower( double Gr ) {
        GR.setPower(Gr);
    }

    public void WbPower( double Wb ) {
        VB.setPower(Wb);
    }

    public void UpPower( double Up  ) {
        UP.setPower(Up);
    }

    /****************SUPPORT_SERVOS_FUNCTIONS******************/

    public void wobblePos( double x ) {
        servoWobble.setPosition(x);
    }

    public void pullPos( double x ) {
        servoPull.setPosition(x);
    }

    /****************SUPPORT_ANGLE_FUNCTIONS******************/

    public void resetAngle() {
        StartAngle += getAngle();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    /****************SUPPORT_IMAGE_FUNCTIONS******************/

    public void testEnc() {
        tele.addData("LF", RA.parssq(LF.getCurrentPosition()));
        tele.addData("RF", RA.parssq(LB.getCurrentPosition()));
        tele.addData("SH", SH.getCurrentPosition());
        tele.addData("VB", VB.getCurrentPosition());
        tele.addData("UP", UP.getCurrentPosition());
        tele.addData("angle", RobotAngle - getAngle());

        tele.addData("Button", button.isPressed() ? "Pressed" : "Not pressed");

        if (gamepad1.b)
        {
            motorsSetPower(1, 1, 1, 1);
            delay(1000);
            motorsSetPower(0, 0, 0, 0);
        }

        tele.update();
    }

    /**********************************FILE_WRITE_FUNCTIONS**********************************/

    /* заполнение массива нулями */

    void setNull (){
        for (int j = 0; j < 10000; j++){
            speedVel[j] = 0;
            deltaSpeedVel[j] = 0;
            averageSpeedVel[j] = 0;
            volt[j] = 0;
            time[j] = 0;
            masx[j] = 0;
            masy[j] = 0;
            masang[j] = 0;
        }
        i = 0;
        RobotAngle = 0;
        resetAngle();
    }

    /* создание файла */

    public void fileWrite() {
        try {
            File f = new File(Environment.getExternalStorageDirectory().toString() + "/" + "ShootTable");
            f.createNewFile();                                         // Создается файл, если он не был создан
            FileOutputStream outputStream = new FileOutputStream(f);   // После чего создаем поток для записи
            for(int j = 0; j < i; j++) {
                outputStream.write((Double.toString(time[j]) + " " + Double.toString(speedVel[j]) + " " + Double.toString(volt[j]) + "00000" + "\n").getBytes());                            // и производим непосредственно запись
            }
            outputStream.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

    }

    Thread shootWrite = new Thread() {
        @Override
        public void run() {
            while (L.opModeIsActive() && !L.isStopRequested()) {
                i++;
                oldtime = L.getRuntime();
                v = voltageSensor.getVoltage();

                speedVel[i] = speedShv;
                deltaSpeedVel[i] = deltaSpeedShv;
                averageSpeedVel[i] = averageSpeedShv;
                volt[i] = v;
                time[i] = oldtime;

                delay(10);
            }
        }
    };

    void GoTo(double x, double y, RunMode RM) {
        RA.GoTo(x, y, this, RM);
    }
    void turnTo(double x) {
        RA.turnTo(x, this);
    }
    void turnToMin(double x) {
        RA.turnToMin(x, this);
    }
    void wobbleAuto() {
        RA.wobbleAuto(this);
    }
    void pushRing() {
        RA.pushRing(this);
    }
    void autoSet(){ RA.autoSet(this); }
    void autoSetTurnTo(){
        RA.autoSetTurnTo(this);
    }
    void autoReleSet(){ RA.autoReleSet(this); }
    void VuforiaInit(){
        RA.VuforiaInit(this);
    }
    Bitmap getImage() throws InterruptedException { return RA.getImage(); }
    int colorAnalyser(Bitmap bm) {
        return RA.colorAnalyser(bm, this);
    }
    void holdLift(){
        RA.holdLift(this, dashboard);
    }
    Thread holdLift = new Thread() {
        @Override
        public void run() {
            holdLift();
        }
    };

    Thread drawSpeed = new Thread() {
        @Override
        public void run() {
            while (L.opModeIsActive() && !L.isStopRequested()) {
                L.resetStartTime();

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("speed", SH.getVelocity());
                dashboard.sendTelemetryPacket(packet);

                L.getRuntime();
            }
        }
    };
     Thread drawY = new Thread() {
         @Override
         public void run() {
             while (L.opModeIsActive() && !L.isStopRequested()) {
                 L.resetStartTime();

                 TelemetryPacket packet = new TelemetryPacket();
                 packet.put("Y", Gr.Y);
                 dashboard.sendTelemetryPacket(packet);

                 L.getRuntime();
             }
         }
     };

     public void ShootGraphics() {
         RT.ShootGraphics(this, dashboard);
     }

     public void TOmniWB() {
         RT.TOmniWB(this, dashboard);
     }
     Thread TOmniWB = new Thread() {
         @Override
         public void run() {
             TOmniWB();
         }
     };

     public void teleMove() {
         RT.teleMove(this);
     }
 }