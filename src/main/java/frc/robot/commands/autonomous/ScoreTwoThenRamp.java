// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoThenRamp extends SequentialCommandGroup {
  /** Creates a new ScoreTwo. */
  public ScoreTwoThenRamp() {
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
      {247.0603526170753, 10.807950229342723}
    };
    double[] headings2 = new double[] {
      150, 180
    };

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetOdometry(250, 30, 180),
      new DriveSwerveProfile(waypoints, headings, 0.4),
      new Wait(1000),
      new DriveSwerveProfile(waypoints2, headings2, 0.4),
      new DriveToPose(240, -60, 180, 0.4),
      new DriveOnRamp(true)
    );
  }
}
