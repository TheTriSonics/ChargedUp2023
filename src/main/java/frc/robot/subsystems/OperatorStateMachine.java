// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.mechanical.HorizontalLiftSubsystem;
import frc.robot.subsystems.mechanical.Pneumatics;
import frc.robot.subsystems.mechanical.VerticalLiftSubsystem;

public class OperatorStateMachine extends SubsystemBase {
  Timer timer;
  public static final int REST = 0;
  public static final int GAMEPIECEPREP = 1;
  public static final int ENGAGEGAMEPIECE = 2;
  public static final int PREPAREPLACEMENT = 3;
  public static final int PLACEGAMEPIECE = 4;
  public static final int GAMEPIECEPREPSHELF = 5;
  public static final int GAMEPIECEPREPSLIDE = 6;
  public static final int ENGAGEGAMEPIECEFROMSHELF = 7;
  
  boolean cube = false;
  public static final int LOW = 0;
  public static final int MID = 1;
  public static final int HIGH = 2;
  public static final int RIGHTCONE = 4;

  int level = LOW;
  private int state = REST;
  int[] nextState = new int[8];
  boolean disabled = false;
  boolean teleop = false;

  // hLiftDelay, vLiftDelay, wheelDelay, clampDelay, flipperDelay, intakeDelay;
  Delays gamePiecePrepDelay = new Delays(0.5, 0, 0, 0.6, 0.6
  , 0);
  Delays engageGamePieceDelay = new Delays(0.5, 0.2, 0.5, 0, 0, 0.5);
  Delays preparePlacementDelay = new Delays(0.75, 0, 0, 0, 0, 0);
  Delays placeGamePieceDelay = new Delays(0, 0, 0, 0, 0, 0);
  Delays restDelay = new Delays(0, 0.5, 0, 0.5, 0.5, 0.5);
  Delays engageGamePieceFromShelfDelay = new Delays(0, 0.75, 0.5, 0, 0, 0.5);
  Delays[] stateDelays = new Delays[8];
  /** Creates a new OperatorStateMachine. */
  public OperatorStateMachine() {
    nextState[REST] = GAMEPIECEPREP;
    nextState[GAMEPIECEPREP] = ENGAGEGAMEPIECE;
    nextState[ENGAGEGAMEPIECE] = PREPAREPLACEMENT;
    nextState[PREPAREPLACEMENT] = PLACEGAMEPIECE;
    nextState[PLACEGAMEPIECE] = REST;
    nextState[GAMEPIECEPREPSLIDE] = ENGAGEGAMEPIECE;
    nextState[GAMEPIECEPREPSHELF] = ENGAGEGAMEPIECE;
    nextState[ENGAGEGAMEPIECEFROMSHELF] = PREPAREPLACEMENT;
    timer = new Timer();
    timer.start();
    stateDelays[REST] = restDelay;
    stateDelays[GAMEPIECEPREP] = gamePiecePrepDelay;
    stateDelays[ENGAGEGAMEPIECE] = engageGamePieceDelay;
    stateDelays[PREPAREPLACEMENT] = preparePlacementDelay;
    stateDelays[PLACEGAMEPIECE] = placeGamePieceDelay;
    stateDelays[GAMEPIECEPREPSHELF] = gamePiecePrepDelay;
    stateDelays[GAMEPIECEPREPSLIDE] = gamePiecePrepDelay;
    stateDelays[ENGAGEGAMEPIECEFROMSHELF] = engageGamePieceFromShelfDelay;
  }

  public void advanceState() {
    timer.reset();
    state = nextState[state];
    disabled = false;
  }

  public void goHome() {
    state = REST;
    disabled = false;
    timer.reset();
  }

  public void setScoringLevel(int scoringLevel) {
    level = scoringLevel;
    disabled = false;
  }

  public void setTeleop(boolean teleop) {
    this.teleop = teleop;
  }

  public void setGamePiece(boolean cube) {
    this.cube = cube;
  }

  public boolean getGamePiece() {
    return cube;
  }

  public int getScoringLevel() {
    return level;
  }

  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
  }

  public void setState(int state) {
    timer.reset();
    this.state = state;
    disabled = false;
  }

  public boolean readyToPlacePiece() {
    return RobotContainer.horizontalLiftSubsystem.inPosition() && RobotContainer.verticalLiftSubsystem.inPosition();
  }

  @Override
  public void periodic() {
    if (disabled) return;
    double time = timer.get();
    Delays delays = stateDelays[state];
    //SmartDashboard.putNumber("State", state);
    SmartDashboard.putBoolean("Cube", cube);
    //SmartDashboard.putNumber("Scoring Level", level);
    switch(state) {
      case REST: {
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint(HorizontalLiftSubsystem.REST);
        if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint(VerticalLiftSubsystem.REST);
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(0);
        if (time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, false);
        break;
      }
      case GAMEPIECEPREP: {
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint(HorizontalLiftSubsystem.LOW);
        if (time >= delays.vLiftDelay) {
          int height;
          if (level == RIGHTCONE) height = VerticalLiftSubsystem.RIGHTCONE;
          else height = VerticalLiftSubsystem.REST;
          RobotContainer.verticalLiftSubsystem.setSetPoint(height);
        }
        
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(1);
        if (time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, true);
        if (time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, true);
        if (time > 1 && teleop && RobotContainer.intakeSubsystem.getPhotoEye()) advanceState();
        break;
      }
      case GAMEPIECEPREPSHELF: {  // Pick up from double human player station
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint(HorizontalLiftSubsystem.LOW);
        if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint(VerticalLiftSubsystem.SHELF);
        //if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint();
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(1);
        if (time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, true);
        if (time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, true);
        if (time > 1 && teleop && RobotContainer.intakeSubsystem.getPhotoEye()) {
          timer.reset();
          state = ENGAGEGAMEPIECEFROMSHELF;
        }
        break;
      }
      case GAMEPIECEPREPSLIDE: { // Pick up from slide, not used
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint(HorizontalLiftSubsystem.SLIDE);
        if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint(VerticalLiftSubsystem.SLIDE);
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(1);
        if (!cube && time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (cube && time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, false);
        break;
      }
      case ENGAGEGAMEPIECEFROMSHELF:
      case ENGAGEGAMEPIECE: {
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint(HorizontalLiftSubsystem.TRAVELING);
        if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint(VerticalLiftSubsystem.TRAVELING);
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(0);
        if (!cube && time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if (time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, false);
        break;
      }
      case PREPAREPLACEMENT: {
        if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint();
        if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint();
        if (time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(0);
        if (!cube && time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, false);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, false);
        if(time >= delays.intakeDelay) RobotContainer.pneumatics.setValve(Pneumatics.INTAKE, true);
        break;
      }
      case PLACEGAMEPIECE: {
        //if (time >= delays.hLiftDelay) RobotContainer.horizontalLiftSubsystem.setSetPoint();
        //if (time >= delays.vLiftDelay) RobotContainer.verticalLiftSubsystem.setSetPoint();
        if (cube && time >= delays.wheelDelay) RobotContainer.intakeSubsystem.setPower(-1);
        if (time > 1) RobotContainer.intakeSubsystem.setPower(0);
        if (time >= delays.clampDelay) RobotContainer.pneumatics.setValve(Pneumatics.CLAMP, true);
        if (time >= delays.flipperDelay) RobotContainer.pneumatics.setValve(Pneumatics.FLIPPER, true);
        break;
      }
    }
    // This method will be called once per scheduler run
  }

  class Delays {
    double hLiftDelay, vLiftDelay, wheelDelay, clampDelay, flipperDelay, intakeDelay;
    Delays(double hLiftDelay, double vLiftDelay, double wheelDelay, double clampDelay, double flipperDelay, double intakeDelay){
      this.hLiftDelay = hLiftDelay;
      this.vLiftDelay = vLiftDelay;
      this.wheelDelay = wheelDelay;
      this.clampDelay = clampDelay;
      this.flipperDelay = flipperDelay;
      this.intakeDelay = intakeDelay;
    }

  }
}
