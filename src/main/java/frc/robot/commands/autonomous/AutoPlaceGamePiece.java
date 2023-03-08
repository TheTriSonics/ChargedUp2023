// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.state.OperatorState;
import frc.robot.utilities.state.ScoringState;

public class AutoPlaceGamePiece extends CommandBase {
  /** Creates a new AutoPlaceGamePiece. */
  Timer timer = new Timer();
  boolean cube;
  ScoringState level;
  final int RAISE = 0;
  final int PLACE = 1;
  final int END = 2;
  int state = RAISE;
  boolean finished = false;
  public AutoPlaceGamePiece(boolean cube, ScoringState level) {
    this.cube = cube;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.operatorStateMachine.setGamePiece(cube);
    RobotContainer.operatorStateMachine.setScoringLevel(level);;
    RobotContainer.operatorStateMachine.setState(OperatorState.GAME_PIECE_PREP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case(RAISE): {
        if (RobotContainer.operatorStateMachine.readyToPlacePiece()) {
          state = PLACE;
          RobotContainer.operatorStateMachine.advanceState();
          timer.start();
        }
        break;
      }
      case(PLACE): {
        if (timer.hasElapsed(0.5)) {
          state = END;
          RobotContainer.operatorStateMachine.advanceState();
          timer.reset();
        }
        break;
      }
      case(END): {
        if (level == ScoringState.LOW || timer.hasElapsed(1.5)) {
          finished = true;
        }
        break;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
