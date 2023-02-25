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
public class CenterLeaveCommunity extends InitializedCommandGroup {
  /** Creates a new CenterLeaveCommunity. */
  public CenterLeaveCommunity() {}
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    addCommands(
      new SetOdometry(250, -32, 180),
      new DriveSwerveProfile(AutonomousProfiles.centerLeaveCommunityGetPiece.get(matchData), 0.25),
      new Wait(1000),
      new DriveSwerveProfile(AutonomousProfiles.centerLeaveCommunityToRamp.get(matchData), 0.25),
      new DriveOnRamp(false)
    );
  }
}
