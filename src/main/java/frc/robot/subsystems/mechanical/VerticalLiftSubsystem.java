// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class VerticalLiftSubsystem extends SubsystemBase {
  public enum LiftHeight {
    FLOOR(0), LOW(10), MID(20), HIGH(50), PLAYER(100);

    public final int counts;
    private LiftHeight(int counts) {
      this.counts = counts;
    }
  }
  private final int MAX_LIFT_ENCODER_TICKS = 4023;
  private final int MIN_LIFT_ENCODER_TICKS = 0; 
  
  private final TalonFX m_leftLiftMotor = new TalonFX(RobotConstants.LEFT_LIFT_MOTOR);
  private final TalonFX m_rightLiftMotor = new TalonFX(RobotConstants.RIGHT_LIFT_MOTOR);
  private int m_targetCounts = 0;

  /** Creates a new VerticalLiftSubsystem. */

  public VerticalLiftSubsystem() {
    m_rightLiftMotor.setInverted(true);
    m_rightLiftMotor.set(TalonFXControlMode.Follower, RobotConstants.LEFT_LIFT_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPosition(LiftHeight height) {
    this.m_targetCounts = height.counts;
  }
}
