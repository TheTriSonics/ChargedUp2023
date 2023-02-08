// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoopyPathToChargeStation extends SequentialCommandGroup {
  double[][] waypoints = new double[][] {
      { 250.0, 30.0 },
      { 150.0, 30.0 },
      { 83.79310889625387, 18.953645546150465 },
      { 83.09483302485617, -25.39072433454289 },
      { 114.51724723775244, -28.96688319588913 }
  };
  double[] headings = new double[] {
      180, 180, 180, 180
  };

  /** Creates a new LoopyPathToChargeStattion. */
  public LoopyPathToChargeStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetOdometry(250, 30, 180),
        new DriveSwerveProfile(waypoints, headings, 0.3),
        new DriveOnRampFromNearSide());
  }
}
