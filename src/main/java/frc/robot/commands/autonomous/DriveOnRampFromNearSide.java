// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;

public class DriveOnRampFromNearSide extends CommandBase {
  /** Creates a new DriveOnRampFromNearSide. */
  double targetX = 177;
  //double targetX = 180;
  double deltaX = 100;

  public DriveOnRampFromNearSide() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setFieldRelative(true);
    RobotContainer.poseEstimator.setPose(257, -51, 180);
    //RobotContainer.poseEstimator.setPose(99, -51, 180);
    //RobotContainer.gyro.setInitialHeading(180);
  }

  double maxSpeed = 0.3;
  double rampDown = 6;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = RobotContainer.poseEstimator.getPose();
    deltaX = targetX - pose.getX();
    if(deltaX > rampDown) {
      RobotContainer.swerveDrive.drive(maxSpeed * RobotData.maxSpeed, 0, 0);
      return;
    }
    if(Math.abs(deltaX) <= rampDown) {
      double xSpeed = maxSpeed/rampDown * deltaX * RobotData.maxSpeed;
      RobotContainer.swerveDrive.drive(xSpeed, 0, 0);
      return;
    } 
    
    RobotContainer.swerveDrive.drive(-maxSpeed * RobotData.maxSpeed, 0, 0);
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(deltaX) < 2;
  }
}
