// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class HorizontalLiftSubsystem extends SubsystemBase {
  private final int MAX_SLIDE_ENCODER_TICKS = 4023;
  private final int MIN_SLIDE_ENCODER_TICKS = 0; 
  
  private final TalonFX m_slideMotor = new TalonFX(RobotConstants.SLIDE_MOTOR);
  private final Solenoid m_slideSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.SLIDE_SOLENOID);

  /** Creates a new HorizontalLiftSubsystem. */
  public HorizontalLiftSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    // TODO: need to limit based on encoder counts
    m_slideMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setSolenoid(boolean out) {
    m_slideSolenoid.set(out);
  }
}
