// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.utilities.state.ScoringState;

public class VerticalLiftSubsystem extends SubsystemBase {
  public static final double MAX_VERTICAL_IN_INCHES = 48.00;
  static final double INCHESPERPULSE = 49.25 / 107638;
  double[] cubeSetPoints = new double[] {
      0, 23.8, 36, 5
  };
  double[] coneSetPoints = new double[] {
      0, 38, 49.5, 5
  };
  double[] setPoints = cubeSetPoints;

  private final TalonFX m_leftLiftMotor = new TalonFX(RobotConstants.LEFT_LIFT_MOTOR);
  private final TalonFX m_rightLiftMotor = new TalonFX(RobotConstants.RIGHT_LIFT_MOTOR);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(40, 100);
  ProfiledPIDController controller = new ProfiledPIDController(0.4, 0, 0.00, constraints, 0.02);

  double setPoint = 0;

  /** Creates a new VerticalLiftSubsystem. */

  public VerticalLiftSubsystem() {
    m_leftLiftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightLiftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightLiftMotor.setInverted(true);
    m_rightLiftMotor.set(TalonFXControlMode.Follower, RobotConstants.LEFT_LIFT_MOTOR);
  }

  @Override
  public void periodic() {
    /*
     * if (RobotContainer.operator.getHID().getBButton()) controller.setGoal(30);
     * if (RobotContainer.operator.getHID().getXButton()) controller.setGoal(10);
     * if (RobotContainer.operator.getHID().getYButton()) controller.setGoal(0);
     */
    double power = -RobotContainer.operator.getLeftY();
    if (Math.abs(power) < 0.1) {
      power = controller.calculate(getPosition());
    } else {
      controller.reset(getPosition());
      controller.setGoal(getPosition());
      RobotContainer.operatorStateMachine.setDisabled(true);
    }
    setPower(power);
    SmartDashboard.putNumber("V Slide Power", power);
    SmartDashboard.putNumber("V Slide", getPosition());
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return m_leftLiftMotor.getSelectedSensorPosition() * INCHESPERPULSE;
  }

  public void resetPosition() {
    m_leftLiftMotor.setSelectedSensorPosition(0);
  }

  public void setPower(double power) {
    if (power > 0 && getPosition() > MAX_VERTICAL_IN_INCHES) {
      power = 0;
    } else if (power < 0 && getPosition() < 0.1) {
      power = 0;
    }
    if (getPosition() < 15 && RobotContainer.pneumatics.getFlipperOut() && power < 0)
      power = 0;
    m_leftLiftMotor.set(TalonFXControlMode.PercentOutput, power * 0.4);
  }

  public void setSetPoint(ScoringState setPoint) {
    this.setPoint = setPoints[setPoint.stateId];
    controller.setGoal(this.setPoint);
  }

  public void setSetPoint() {
    boolean cube = RobotContainer.operatorStateMachine.getGamePiece();
    int level = RobotContainer.operatorStateMachine.getScoringLevel();
    if (cube) {
      setPoints = cubeSetPoints;
    } else {
      setPoints = coneSetPoints;
    }
    this.setPoint = setPoints[level];
    controller.setGoal(this.setPoint);
  }

  public boolean inPosition() {
    return Math.abs(getPosition() - setPoint) < 3;
  }
}
