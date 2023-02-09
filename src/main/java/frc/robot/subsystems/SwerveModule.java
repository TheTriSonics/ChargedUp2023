// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class SwerveModule {
  private static final double kWheelRadius = 2; // inches
  private static final double kDriveGearRatio = 7.131;
  private static final double kEffectiveRadius = kWheelRadius / kDriveGearRatio;
  private static final int kDriveResolution = 2048;
  private static final int kTurnResolution = 4096;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private AnalogInput m_turnEncoder;
  private int m_encoderOffset = 0;
  private final PIDController m_turnPIDController = new PIDController(1 / 1000.0, 0, 0);

  boolean driveDisabled = false;
  String name;

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public SwerveModule(int driveMotorID, int turningMotorID, int turnEncoderChannel, int turnEncoderoffset, String name,
      boolean driveDisabled) {
    this(driveMotorID, turningMotorID, turnEncoderChannel, turnEncoderoffset, name);
    this.driveDisabled = driveDisabled;
  }

  public SwerveModule(int driveMotorID, int turningMotorID, int turnEncoderChannel, int turnEncoderoffset,
      String name) {
    // TODO: The controllers won't be SparkMax, this needs to change.
    m_driveMotor = new TalonFX(driveMotorID);
    m_turningMotor = new TalonFX(turningMotorID);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_encoderOffset = turnEncoderoffset;
    m_driveMotor.config_kF(0, 0.0465);
    m_driveMotor.config_kP(0, 0.0);
    m_turningMotor.config_kP(0, 0.1);
    m_turningMotor.config_kD(0, 0.002);
    m_turningMotor.setInverted(TalonFXInvertType.Clockwise);

    m_turnEncoder = new AnalogInput(turnEncoderChannel);

    this.name = name;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDriveVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * 10 / kDriveResolution * 2 * Math.PI * kEffectiveRadius;
  }

  public double getTurnPositionInradians() {
    return getTurnPosition() / kTurnResolution * 2 * Math.PI;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModulePosition getState() {
    double distance = m_driveMotor.getSelectedSensorPosition() / kDriveResolution * 2 * Math.PI * kEffectiveRadius;
    return new SwerveModulePosition(distance, new Rotation2d(getTurnPositionInradians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double currentTurnPosition = getTurnPosition();
    double turnAngle = 2 * Math.PI / kTurnResolution * currentTurnPosition;
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnAngle));

    double driveVelocity = state.speedMetersPerSecond / (2 * Math.PI * kEffectiveRadius) * kDriveResolution * 0.1;
    double setPosition = state.angle.getRadians() / (2 * Math.PI) * kTurnResolution;
    double revs = Math.round(currentTurnPosition / kTurnResolution);
    setPosition += revs * kTurnResolution;
    while (setPosition > currentTurnPosition + kTurnResolution / 2)
      setPosition -= kTurnResolution;
    while (setPosition < currentTurnPosition - kTurnResolution / 2)
      setPosition += kTurnResolution;

    if (!driveDisabled)
      m_driveMotor.set(TalonFXControlMode.Velocity, driveVelocity);
    m_turnPIDController.setSetpoint(setPosition);
    double turnPower = m_turnPIDController.calculate(currentTurnPosition);
    m_turningMotor.set(TalonFXControlMode.PercentOutput, turnPower);

  }

  public void resetTurnEncoder() {
    m_encoderOffset = m_turnEncoder.getValue();
  }

  public double getTurnPosition() {
    return m_turnEncoder.getValue() - m_encoderOffset;
  }

  public void stopDriveMotor() {
    m_driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    m_turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

}
