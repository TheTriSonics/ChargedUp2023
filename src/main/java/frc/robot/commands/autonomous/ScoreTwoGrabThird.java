// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.RobotContainer;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoGrabThird extends InitializedCommandGroup {
  /** Creates a new ScoreTwoGrabThird. */
  public ScoreTwoGrabThird() {}

  public void initialization(){
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    PickOneGamePiece pogp = new PickOneGamePiece();
    pogp.initialization();
    
    addCommands(
      pogp,
      new DriveSwerveProfile(AutonomousProfiles.secondToThirdGamepiece.get(matchData), 0.4),
      new DriveSwerveProfile(AutonomousProfiles.thirdPiecePlacement.get(matchData), 0.4)
    );
  }
}

  