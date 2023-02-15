// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoGrabThird extends SequentialCommandGroup {
  /** Creates a new ScoreTwoGrabThird. */
  public ScoreTwoGrabThird() {
      double[][] waypoints = new double[][] {
        { 250.0, 30.0 },
        { 190.87931749063435, 30.397353902458427 },
        { 150.5344885237755, 30.10596326884294 },
        { 67.03448798270918, 20.390731496573693 }
      };
      double[] headings = new double[] {
        100, 60, 0
      };
      double[][] waypoints2 = new double[][] {
        {67.03448798270918, 20.390731496573693},
        {152.92242016462566, 17.523182001611968},
        {250.0603526170753, 10.807950229342723}
      };
      double[] headings2 = new double[] {
        90, 180
      };
      double[][] waypoints3 = new double[][] {
        {251.4, 16.8},
        {150.1, 31.1},
        {92.2, 13.9},
        {66.3, -24.7}
      };
      double[] headings3 = new double[] {
        180, 90, 0
      };
  
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        new SetOdometry(250, 30, 180),
        new DriveSwerveProfile(waypoints, headings, 0.3),
        new Wait(1000),
        new DriveSwerveProfile(waypoints2, headings2, 0.3),
        new DriveSwerveProfile(waypoints3, headings3, 0.2)
      );
    }
  }

  