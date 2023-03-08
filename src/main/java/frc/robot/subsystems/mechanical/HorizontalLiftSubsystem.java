// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.utilities.state.ScoringState;

public class HorizontalLiftSubsystem extends SubsystemBase {
  public static final double MAX_HORIZONTAL_IN_INCHES = 43.5;
  double[] cubeSetPoints = new double[] {
      0, 20, 36, 0
  };
  double[] coneSetPoints = new double[] {
      0, 21.5, 40.2, 0
  };

  double[] setPoints = cubeSetPoints;

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(50, 200);
  ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0.00, constraints, 0.02);

  private final TalonFX m_slideMotor = new TalonFX(RobotConstants.SLIDE_MOTOR);

  double setPoint = 0;
  final double INCHESPERPULSE = 43.375 / 251813;

  /** Creates a new HorizontalLiftSubsystem. */
  public HorizontalLiftSubsystem() {
  }

  @Override
  public void periodic() {
    /*
     * if (RobotContainer.operator.getHID().getBButton()) controller.setGoal(30);
     * if (RobotContainer.operator.getHID().getXButton()) controller.setGoal(10);
     * if (RobotContainer.operator.getHID().getYButton()) controller.setGoal(0);
     */
    // This method will be called once per scheduler run
    double power = -RobotContainer.operator.getRightY();
    double position = getPosition();
    if (Math.abs(power) < 0.1) {
      power = controller.calculate(position);
    } else {
      controller.reset(position);
      controller.setGoal(position);
      RobotContainer.operatorStateMachine.setDisabled(true);
    }
    setPower(power);
    SmartDashboard.putNumber("H Slide Power", power);
    SmartDashboard.putNumber("H Slide", getPosition());
  }

  public void setPower(double power) {
    if (power > 0 && getPosition() > MAX_HORIZONTAL_IN_INCHES) {
      power = 0;
    } else if (power < 0 && getPosition() < 0.1) {
      power = 0;
    }
    m_slideMotor.set(TalonFXControlMode.PercentOutput, power);
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

  public double getPosition() {
    return m_slideMotor.getSelectedSensorPosition() * INCHESPERPULSE;
  }

  public void resetPosition() {
    m_slideMotor.setSelectedSensorPosition(0);
  }

  public boolean inPosition() {
    return Math.abs(getPosition() - setPoint) < 3;
  }
}
