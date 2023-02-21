// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.RobotContainer;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoThenLoopyToRamp extends InitializedCommandGroup {
  /** Creates a new ScoreTwoThenLoopyToRamp. */
  public ScoreTwoThenLoopyToRamp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    PickOneGamePiece pogp = new PickOneGamePiece();
    pogp.initialization();
    
    addCommands(
      pogp,
      new DriveSwerveProfile(AutonomousProfiles.loopyPath.get(matchData), 0.4),
      new DriveOnRamp(false)
    );
  }
}
