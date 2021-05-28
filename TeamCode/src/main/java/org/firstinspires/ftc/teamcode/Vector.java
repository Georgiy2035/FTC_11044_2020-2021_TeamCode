package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Vector {
    double X, Y;
    Vector (double X, double Y) {
        this.X = X;
        this.Y = Y;
    }
    void set(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }
    Vector sum(Vector A){
        return new Vector(this.X + A.X, this.Y + A.Y);
    }
    Vector dif(Vector A){
        return new Vector (this.X - A.X, this.Y - A.Y);
    }

    Vector rotate(double alpha) {
        double X1 = this.X * cos(alpha / 180 * Math.PI) + this.Y * sin(alpha / 180 * Math.PI);
        double Y1 = this.Y * cos(alpha / 180 * Math.PI) - this.X * sin(alpha / 180 * Math.PI);
        return new Vector(X1, Y1);
    }
}
