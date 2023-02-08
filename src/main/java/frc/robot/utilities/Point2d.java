// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class Point2d {
    public double x, y;
    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point2d(double p[]) {
        this.x = p[0];
        this.y = p[1];
    }

    public double length() {
        return Math.sqrt(x*x + y*y);
    }

    public String toString() {
        return x + " " + y;
    }
}
