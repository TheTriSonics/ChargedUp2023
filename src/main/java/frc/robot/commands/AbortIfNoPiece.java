// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.SetAutonState;
import frc.robot.subsystems.AutonStateMachine.State;

public class AbortIfNoPiece extends CommandBase {
  public AbortIfNoPiece() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.intakeSubsystem.getPhotoEye()) {
      RobotContainer.autonStateMachine.setState(State.FIRST_LEG_COMPLETE);
    } else {
      RobotContainer.autonStateMachine.setState(State.ABORT_PICKUP);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
