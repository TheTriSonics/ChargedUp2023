// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.subsystems.OperatorStateMachine;
import frc.robot.subsystems.AutonStateMachine.State;

public class ScoreTwoWithAbort extends CommandBase {

  public ScoreTwoWithAbort() {}

  private boolean firstRun = true;
  private boolean finished = true;
  private String matchData;
  private double[] odometry;

  public void initilize() {
    matchData = RobotContainer.getMatchData();
    odometry = AutonomousProfiles.initialOdometries.get(matchData);
  }

  public void execute() {
    if (firstRun) {
      firstRun = false;
      SequentialCommandGroup firstLeg = new SequentialCommandGroup(
        Commands.parallel(
          new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
          new SetOdometry(odometry[0], odometry[1], odometry[2]),
          new SetUseAprilTags(false)
        ),
        new ParallelCommandGroup(
          new DriveSwerveProfile(AutonomousProfiles.driveToFirstGamePiece.get(matchData), 0.3, true),// 0.6), 
          Commands.parallel(
            Commands.sequence(new Wait(1800), new AdvanceState())),
            new SetGamePiece(true),
            new SetScoringLevel(OperatorStateMachine.HIGH)
          ),
        new AbortIfNoPiece()
      );
      firstLeg.schedule();
    }
    else if (!firstRun && RobotContainer.autonStateMachine.getState() == State.PROCEED) {
      // Do nothing. Just chill, or vibe as the whippersnappers say
    }
    else if (!firstRun && RobotContainer.autonStateMachine.getState() == State.FIRST_LEG_COMPLETE) {
      // Go place the pice we've successfully captured
      SequentialCommandGroup placePiece = new SequentialCommandGroup(
        new AdvanceState(),
        new Wait(500),
        new SetGamePiece(true),
        new DriveSwerveProfile(AutonomousProfiles.firstGamePieceToSecondPlacement.get(matchData), 0.35), //0.6));
        Commands.parallel(
          new AdvanceState(),
          new InScoringPosition()
        ),
        new AdvanceState(),
        new Wait(300),
        new AdvanceState(),
        new Wait(1000)
      );
      placePiece.schedule();
      finished = true;
    }

    else if (!firstRun && RobotContainer.autonStateMachine.getState() == State.ABORT_PICKUP) {
      // Drive to ramp?
      // For now just sit here.
      finished = true;
    }
  }

  public boolean isFinshed() {
    return finished;
  }
}
