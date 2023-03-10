// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private final Solenoid m_clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.CLAMP_SOLENOID);
  private final Solenoid m_flipperSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.FLIPPER_SOLENOID);
  private final Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.INTAKE_SOLENOID);
  public static final int INTAKE = 0;
  public static final int CLAMP = 1;
  public static final int FLIPPER = 2;
  Solenoid[] valves = new Solenoid[] {m_intakeSolenoid, m_clampSolenoid, m_flipperSolenoid};
  public Pneumatics() {
    m_clampSolenoid.set(false);
    m_flipperSolenoid.set(false);
    m_intakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Flipper", getFlipperOut());
    /*
    if (RobotContainer.operator.getHID().getRightBumper()) m_intakeSolenoid.set(true);
    if (RobotContainer.operator.getHID().getLeftBumper()) m_intakeSolenoid.set(false);
    
    if (RobotContainer.operator.getHID().getRightBumper()) m_clampSolenoid.set(true);
    if (RobotContainer.operator.getHID().getLeftBumper()) m_clampSolenoid.set(false);
    if (RobotContainer.operator.getHID().getYButton()) m_flipperSolenoid.set(true);
    if (RobotContainer.operator.getHID().getAButton()) m_flipperSolenoid.set(false);
    */

    // This method will be called once per scheduler run
  }

  public void setValve(int valve, boolean state) {
    // if (valve == CLAMP) state = !state;
    valves[valve].set(state);
  }

  public boolean getFlipperOut() {
    return m_flipperSolenoid.get();
  }

  public boolean getIntakeOut() {
    return m_intakeSolenoid.get();
  }
}
