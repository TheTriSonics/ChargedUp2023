// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;

public class HorizontalLiftSubsystem extends SubsystemBase {
  public static final int LOW = 0;
  public static final int MID = 1;
  public static final int HIGH = 2;
  public static final int TRAVELING = 3;
  public static final int REST = 4;
  public static final int SHELF = 5;
  public static final int SLIDE = 6;
  public static final int RIGHTCONE = LOW;

  public static final double MAX_HORIZONTAL_IN_INCHES = 43.5;
  double[] cubeSetPoints = new double[] {
    5, 20, 36, 0, 0, 0, 0 // 36
  };
  double[] coneSetPoints = new double[] {
    5, 21.5, 38, 0, 0, 0, 0 // was 40.2
  };

  double[] setPoints = cubeSetPoints;

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(50, 200);
  //TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(75, 120);
  ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0.00, constraints, 0.02);
  //ProfiledPIDController controller = new ProfiledPIDController(0.15, 0, 0.00, constraints, 0.02);
  
  private final TalonFX m_slideMotor = new TalonFX(RobotConstants.SLIDE_MOTOR);
  private final Encoder encoder = new Encoder(0, 1);

  double setPoint = 0;
  final double INCHESPERPULSE = 43.375/251813; // for the Talon
  final double ENCODERINCHESPERPULSE = INCHESPERPULSE;// * 106661 / 5352.75;

  /** Creates a new HorizontalLiftSubsystem. */
  public HorizontalLiftSubsystem() {
    encoder.setReverseDirection(true);
    
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("H Slide Position", getPosition());
    
    // This method will be called once per scheduler run
    double power = -0.75*RobotContainer.operator.getRightY();
    double position = getPosition();
    if (Math.abs(power) < 0.1) {
      power = controller.calculate(position);
    } else {
      controller.reset(position);
      controller.setGoal(position);
      RobotContainer.operatorStateMachine.setDisabled(true);
    }
    setPower(power);
    //SmartDashboard.putNumber("H Slide Power", power);
    //SmartDashboard.putNumber("H Slide", getPosition());
  }

  public void setPower(double power) {
    double origPower = power;
    if (power > 0 && getPosition() >  MAX_HORIZONTAL_IN_INCHES) {
      power = 0;
    }
    else if (power < 0 && getPosition() < 3) { // JJB: Altered from 0.1 to 0.5 during STandish at 12:37pm.
      power = Math.max(power, -0.05*getPosition());
    }
    //SmartDashboard.putNumber("Orig Power", origPower);
    //SmartDashboard.putNumber("Final Power", power);
    m_slideMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setSetPoint(int setPoint) {
    this.setPoint = setPoints[setPoint];
    controller.setGoal(this.setPoint);
  }

  public void setSetPoint() {
    boolean cube = RobotContainer.operatorStateMachine.getGamePiece();
    int level = RobotContainer.operatorStateMachine.getScoringLevel();
    if (cube) setPoints = cubeSetPoints;
    else setPoints = coneSetPoints;
    this.setPoint = setPoints[level];
    controller.setGoal(this.setPoint);
  }

  public double getPosition() {
    return m_slideMotor.getSelectedSensorPosition() * INCHESPERPULSE * 24/18.0;
    //return encoder.getDistance() * ENCODERINCHESPERPULSE;
  }

  public void resetPosition() {
    m_slideMotor.setSelectedSensorPosition(0);
  }

  public boolean inPosition() {
    return Math.abs(getPosition() - setPoint) < 1.5;
  }
}
