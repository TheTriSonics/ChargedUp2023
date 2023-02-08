// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class BezierCurve {
    Point2d P0, P1, P2, P3;
    public BezierCurve(Point2d P0, Point2d P1, 
                       Point2d P2, Point2d P3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
    }

    public Point2d getPosition(double s) {
        Point2d result = Math2d.smult(Math.pow(1-s,3), P0);
        result = Math2d.sum2d(result, Math2d.smult(3*s*(1-s)*(1-s),P1));
        result = Math2d.sum2d(result, Math2d.smult(3*s*s*(1-s), P2));
        return Math2d.sum2d(result, Math2d.smult(Math.pow(s, 3), P3));
    }

    public Point2d getTangent(double s) {
        Point2d result = Math2d.smult(-3*(1-s)*(1-s), P0);
        result = Math2d.sum2d(result, Math2d.smult(9*s*s-12*s+3, P1));
        result = Math2d.sum2d(result, Math2d.smult(6*s-9*s*s, P2));
        return Math2d.sum2d(result, Math2d.smult(3*s*s, P3));
    }

    public double length() {
        double l = 0;
        int N = 100;
        double s = 0;
        double ds = 1/(float)N;
        for (int i = 0; i < N; i++) {
            l += getTangent(s).length() * ds;
            s += ds;
        }
        return l;
    }

    public void print() {
        System.out.println(P0.x + " " + P0.y);
        System.out.println(P1.x + " " + P1.y);
        System.out.println(P2.x + " " + P2.y);
        System.out.println(P3.x + " " + P3.y);
    }
}
