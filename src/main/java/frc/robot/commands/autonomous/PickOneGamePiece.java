// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickOneGamePiece extends SequentialCommandGroup {
  double[][] waypoints = new double[][]{
    {250.0, 30.0},
    {210.87931749063435, 30.397353902458427},
    {107.5344885237755, 26.10596326884294},
    {67.03448798270918, 25.390731496573693}
  };
  double[] headings = new double[]{
     120, 60, 0
  };
  double[][] waypoints2 = new double[][]{
    {67.03448798270918, 25.390731496573693},
    {107.5344885237755, 26.10596326884294},
    {210.87931749063435, 30.397353902458427},
    {250.0, 30.0}
  };
  double[] headings2 = new double[]{
     120, 180, 180
  };
  /** Creates a new PickOneGamePiece. */
  public PickOneGamePiece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetOdometry(250, 30, 180),
      new DriveSwerveProfile(waypoints, headings, 0.30), new Wait(1000),
      new DriveSwerveProfile(waypoints2, headings2, 0.30)
    );
  }
}
