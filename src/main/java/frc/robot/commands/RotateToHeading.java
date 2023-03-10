// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;

public class RotateToHeading extends CommandBase {
  /** Creates a new RotateToHeading. */
  double target;
  double angleError;
  
  public RotateToHeading(double heading) {
    target = heading;
    addRequirements(RobotContainer.swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleError = 10000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = RobotContainer.poseEstimator.getPose().getRotation().getDegrees();
    angleError = target - heading;
    while (angleError > 180) angleError -= 360;
    while (angleError < -180) angleError += 360;
    double rotSpeed = angleError / 90 * RobotData.maxAngularSpeed;
    RobotContainer.swerveDrive.drive(0, 0, rotSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleError) < 5;
  }
}
