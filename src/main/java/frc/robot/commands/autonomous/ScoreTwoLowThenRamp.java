// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.subsystems.OperatorStateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoLowThenRamp extends InitializedCommandGroup {
  /** Creates a new ScoreTwo. */
  public ScoreTwoLowThenRamp() {}

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    double driveToX = matchData.startsWith("R") ? 245:-245;
    double driveToHeading = matchData.startsWith("R") ? 180:-0;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      Commands.parallel(
        new SetGamePiece(false),
        new SetScoringLevel(OperatorStateMachine.LOW),
        new SetOperatorState(OperatorStateMachine.PREPAREPLACEMENT),
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new Wait(800),
      new AdvanceState(),
      new ParallelCommandGroup(
        new DriveSwerveProfile(AutonomousProfiles.driveToFirstGamePiece.get(matchData), 0.3),// 0.6), 
        Commands.sequence(new Wait(500), new GoHome()),
        Commands.parallel(
          Commands.sequence(new Wait(2000), new AdvanceState()),
          new SetGamePiece(true),
          new SetScoringLevel(OperatorStateMachine.LOW)
        )
      ),
      new AdvanceState(),  // engaged
      new Wait(200),
      
      new DriveSwerveProfile(AutonomousProfiles.firstGamePieceToSecondPlacement.get(matchData), 0.3), //0.6));
      new AdvanceState(), // prep
      new Wait(500), 
      new AdvanceState(), // place
      new Wait(500),
      
      Commands.parallel(
        Commands.sequence(new Wait(500), new GoHome()),
        new DriveToPose(driveToX, -60, driveToHeading, 0.4)
      ),
      new DriveOnRamp(true)
      
    );

  }
}
