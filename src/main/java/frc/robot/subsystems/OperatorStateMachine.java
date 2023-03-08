// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.mechanical.Pneumatics;
import frc.robot.utilities.state.OperatorState;
import frc.robot.utilities.state.ScoringState;

public class OperatorStateMachine extends SubsystemBase {

  // Timing variables
  Timer timer;

  // Config variables
  boolean cube = false;

  // State variables
  ScoringState level = ScoringState.LOW;
  OperatorState state = OperatorState.REST;
  OperatorState[] nextState = new OperatorState[5];
  boolean disabled = false;

  // hLiftDelay, vLiftDelay, wheelDelay, clampDelay, flipperDelay, intakeDelay;
  Delays gamePiecePrepDelay = new Delays(0, 0, 0, 0.3, 0.3, 0);
  Delays engageGamePieceDelay = new Delays(0.3, 0.1, 0.5, 0, 0, 0.2);
  Delays preparePlacementDelay = new Delays(0.5, 0, 0, 0, 0, 0);
  Delays placeGamePieceDelay = new Delays(0, 0, 0, 0, 0, 0);
  Delays restDelay = new Delays(0, 0.5, 0, 0, 0, 0.5);
  Delays[] stateDelays = new Delays[5];

  /** Creates a new OperatorStateMachine. */
  public OperatorStateMachine() {
    nextState[0] = OperatorState.GAME_PIECE_PREP;
    nextState[1] = OperatorState.GAME_PIECE_ENGAGE;
    nextState[2] = OperatorState.GAME_PIECE_EXTEND;
    nextState[3] = OperatorState.GAME_PIECE_PLACE;
    nextState[4] = OperatorState.REST;

    timer = new Timer();
    timer.start();
    stateDelays[0] = restDelay;
    stateDelays[1] = gamePiecePrepDelay;
    stateDelays[2] = engageGamePieceDelay;
    stateDelays[3] = preparePlacementDelay;
    stateDelays[4] = placeGamePieceDelay;
  }

  public void advanceState() {
    timer.reset();
    state = nextState[state.stateId];
    disabled = false;
  }

  public void goHome() {
    state = OperatorState.REST;
    disabled = false;
    timer.reset();
  }

  public void setScoringLevel(ScoringState scoringLevel) {
    level = scoringLevel;
  }

  public void setGamePiece(boolean cube) {
    this.cube = cube;
  }

  public boolean getGamePiece() {
    return cube;
  }

  public int getScoringLevel() {
    return level.stateId;
  }

  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
  }

  public void setState(OperatorState state) {
    timer.reset();
    this.state = state;
    disabled = false;
  }

  public boolean readyToPlacePiece() {
    return RobotContainer.horizontalLiftSubsystem.inPosition() && RobotContainer.verticalLiftSubsystem.inPosition();
  }

  @Override
  public void periodic() {
    if (disabled) {
      return;
    }

    double time = timer.get();
    Delays delays = stateDelays[state.stateId];
    SmartDashboard.putNumber("State", state.stateId);
    SmartDashboard.putBoolean("Cube", cube);
    SmartDashboard.putNumber("Scoring Level", level.stateId);

    // If total time for everything to be activated has been reached, don't set
    // anything again.
    if (time > delays.getMaxTime()) {
      return;
    }

    switch (state) {
      case REST: {
        if (time >= delays.hLiftDelay)
          RobotContainer.horizontalLiftSubsystem.setSetPoint(ScoringState.LOW);
        if (time >= delays.vLiftDelay)
          RobotContainer.verticalLiftSubsystem.setSetPoint(ScoringState.LOW);
        if (time >= delays.wheelDelay)
          RobotContainer.intakeSubsystem.setPower(0);
        if (time >= delays.clampDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, false);
        break;
      }
      case GAME_PIECE_PREP: {
        if (time >= delays.hLiftDelay)
          RobotContainer.horizontalLiftSubsystem.setSetPoint(ScoringState.LOW);
        if (time >= delays.vLiftDelay)
          RobotContainer.verticalLiftSubsystem.setSetPoint(ScoringState.LOW);
        if (time >= delays.wheelDelay)
          RobotContainer.intakeSubsystem.setPower(1);
        if (time >= delays.clampDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, true);
        if (time >= delays.intakeDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, true);
        break;
      }
      case GAME_PIECE_ENGAGE: {
        if (time >= delays.hLiftDelay)
          RobotContainer.horizontalLiftSubsystem.setSetPoint(ScoringState.TRAVELING);
        if (time >= delays.vLiftDelay)
          RobotContainer.verticalLiftSubsystem.setSetPoint(ScoringState.TRAVELING);
        if (time >= delays.wheelDelay)
          RobotContainer.intakeSubsystem.setPower(0);
        if (cube == false && time >= delays.clampDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, false);
        break;
      }
      case GAME_PIECE_EXTEND: {
        if (time >= delays.hLiftDelay)
          RobotContainer.horizontalLiftSubsystem.setSetPoint();
        if (time >= delays.vLiftDelay)
          RobotContainer.verticalLiftSubsystem.setSetPoint();
        if (time >= delays.wheelDelay)
          RobotContainer.intakeSubsystem.setPower(0);
        if (time >= delays.clampDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, true);
        break;
      }
      case GAME_PIECE_PLACE: {
        if (cube && time >= delays.wheelDelay)
          RobotContainer.intakeSubsystem.setPower(-1);
        if (time >= delays.clampDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay)
          RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, true);
        break;
      }
    }
  }

  class Delays {
    double hLiftDelay, vLiftDelay, wheelDelay, clampDelay, flipperDelay, intakeDelay;

    Delays(double hLiftDelay, double vLiftDelay, double wheelDelay, double clampDelay, double flipperDelay,
        double intakeDelay) {
      this.hLiftDelay = hLiftDelay;
      this.vLiftDelay = vLiftDelay;
      this.wheelDelay = wheelDelay;
      this.clampDelay = clampDelay;
      this.flipperDelay = flipperDelay;
      this.intakeDelay = intakeDelay;
    }

    public double getMaxTime() {
      double[] delays = new double[] { this.hLiftDelay, this.vLiftDelay, this.wheelDelay, this.clampDelay,
          this.flipperDelay, this.intakeDelay };
      double max = delays[0];
      for (int i = 0; i < delays.length; i++) {
        if (delays[i] > max) {
          max = delays[i];
        }
      }
      return max;
    }

  }
}
