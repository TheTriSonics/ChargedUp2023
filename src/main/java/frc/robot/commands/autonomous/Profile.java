// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

/** Add your docs here. */
public class Profile {
    public double[][] waypoints;
    public double[] headings;
    Profile(double[][] wp, double[] ds) {
        waypoints = wp;
        headings = ds;
    }

}
