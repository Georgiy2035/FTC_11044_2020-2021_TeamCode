package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Const {
    static double KDy = 0.045, KDx = 0.01;

    static double mincorr = 0.9, minx = 0.05, miny = 0.01, mincorra = 0.05;
    static double ticsx = 1230, ticsy = 1630, ang = 30;

    static double KPy = 0.005/*(1 - miny) / ticsy * 0.8 - 0.0001/*0.0008*/, KPx = 0.002;//(1 - minx) / ticsx;
    static double KA = 0.032, KAD = 0.05;

    static double KP = 0.08;//(1 - mincorr) / ang;
    static double KD = 20;
    static double KPm = 0.033, KDm = 0.03;

    /****************TELEMETRY_CONST******************/

    static PIDCoefficients pidWb1 = new PIDCoefficients(0.0021, 0.000000000, 0.000);
    static PIDCoefficients pidWb2 = new PIDCoefficients(0.0032, 0.000000000, 0.000);
    static PIDCoefficients pidUp = new PIDCoefficients(0.0005, 0.000000000, 0.000);
    //static PIDCoefficients pidShStable = new PIDCoefficients(0.00095, 0.0001, 0.00024);
    //static PIDCoefficients pidShStart = new PIDCoefficients(0.009, 0.0001, 0.009);
    static PIDCoefficients pidShStable = new PIDCoefficients(0.0000095, 0.000000012, 0.00085);
    static PIDCoefficients pidShStart = new PIDCoefficients(0.00004, 0.0000005, 0.0008);

    static PIDCoefficients pidShStableA = new PIDCoefficients(0.002,  0.0001, 0.00055);
    static PIDCoefficients pidShStableAt = new PIDCoefficients(0.002,  0.0001, 0.000055);
    static PIDCoefficients pidShStartA = new PIDCoefficients(0.008, 0.000025, 0.0055);//0.005245
    static PIDCoefficients pidShStartAt = new PIDCoefficients(0.0004, 0.000025, 0.005);

    static double goalPos = 1655, targetPos = 1435, targetPosA = 1437, goalPosA = 1560, upPos = 0, wbPos1 = 300, wbPos2 = 0;
}
