// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftIntakeMotor = new CANSparkMax(RobotConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightIntakeMotor = new CANSparkMax(RobotConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
  
  // private final AnalogInput m_photoEye = new AnalogInput(RobotConstants.INTAKE_PHOTO_EYE);
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);
    m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    //m_rightIntakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    double power = 0;
    if (RobotContainer.operator.getHID().getLeftTriggerAxis() >= 0.5) {
      power = 0.5;
    }
    
    if (RobotContainer.operator.getHID().getRightTriggerAxis() >= 0.5) {
      power = -0.5;
    }

    setPower(power);
  }
  
  public void setPower(double power) {
    m_leftIntakeMotor.set(power);
    m_rightIntakeMotor.set(-power);
  }
}
