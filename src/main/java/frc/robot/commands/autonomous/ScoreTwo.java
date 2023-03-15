// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwo extends InitializedCommandGroup {
  /** Creates a new ScoreTwo. */
  public ScoreTwo() {}

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    PickOneGamePiece pogp = new PickOneGamePiece();
    pogp.initialization();
    double driveToX = matchData.startsWith("R") ? 240:-240;
    double driveToHeading = matchData.startsWith("R") ? 180:-0;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      pogp//, 
      //new DriveToPose(driveToX, -60, driveToHeading, 0.4),
      //new DriveOnRamp(true)
    );
  }
}
