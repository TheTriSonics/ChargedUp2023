// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final Spark m_leftIntakeMotor = new Spark(RobotConstants.LEFT_INTAKE_MOTOR);
  private final Spark m_rightIntakeMotor = new Spark(RobotConstants.RIGHT_INTAKE_MOTOR);
  private final AnalogInput m_photoEye = new AnalogInput(RobotConstants.INTAKE_PHOTO_EYE);
  private final Solenoid m_clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.INTAKE_CLAMP_SOLENOID);
  private final Solenoid m_pivotSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.INTAKE_PIVOT_SOLENOID);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
   m_rightIntakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    m_leftIntakeMotor.set(power);
    m_rightIntakeMotor.set(power);
  }
  public void setClamp(boolean open) {
    m_clampSolenoid.set(open);
  }
  public void setPivot(boolean open) {
    m_pivotSolenoid.set(open);
  }
}
