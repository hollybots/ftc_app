package org.firstinspires.ftc.teamcode;

public class FieldPlacement {

    protected double x;
    protected double y;

    protected double theta;
    protected double r;

    protected double orientation;

    FieldPlacement(double x, double y) {

        this.x = x;
        this.y = y;
        r = Math.sqrt(x*x + y*y);
        theta = Math.acos(x/r);

    }

    FieldPlacement(double x, double y, double orientation) {

        this.x = x;
        this.y = y;
        r = Math.sqrt(x*x + y*y);
        theta = Math.acos(x/r);

        this.orientation = orientation;

    }

    FieldPlacement(double r, double theta, boolean vectorial) {

        this.theta = theta;
        this.r = r;
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);

    }

    FieldPlacement(double r, double theta, boolean vectorial, double orientation) {

        this.theta = theta;
        this.r = r;
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);

        this.orientation = orientation;

    }

}
