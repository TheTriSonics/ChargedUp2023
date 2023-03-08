// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AdvanceState;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.InScoringPosition;
import frc.robot.commands.SetGamePiece;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.SetScoringLevel;
import frc.robot.commands.Wait;
import frc.robot.utilities.state.ScoringState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickOneGamePiece extends InitializedCommandGroup {
  
  /** Creates a new PickOneGamePiece. */
  public PickOneGamePiece() {}
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    
    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(false, ScoringState.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new ParallelCommandGroup(
        new DriveSwerveProfile(AutonomousProfiles.driveToFirstGamePiece.get(matchData), 0.3),// 0.6), 
        Commands.parallel(
          Commands.sequence(new Wait(1500), new AdvanceState())),
          new SetGamePiece(true),
          new SetScoringLevel(ScoringState.HIGH)
        ),
        new AdvanceState(),
        new Wait(200),
        new DriveSwerveProfile(AutonomousProfiles.firstGamePieceToSecondPlacement.get(matchData), 0.3), //0.6));
        Commands.parallel(
          new AdvanceState(),
          new InScoringPosition()
        ),
        new AdvanceState(),
        new Wait(300),
        new AdvanceState(),
        new Wait(1000)
    );
  }
}
