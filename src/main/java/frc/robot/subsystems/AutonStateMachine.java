// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonStateMachine extends SubsystemBase {
  public enum State {
    PROCEED,
    FIRST_LEG_COMPLETE,
    ABORT_PICKUP,
  }

  private State state = State.PROCEED;

  public void setState(State state) {
    this.state = state;
  }

  public State getState() {
    return state;
  }
}
