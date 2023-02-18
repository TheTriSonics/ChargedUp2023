// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;

public class DriveOnRamp extends CommandBase {
  /** Creates a new DriveOnRampFromNearSide. */
  double targetX = 177;
  //double targetX = 180;
  double deltaX = 100;
  boolean nearSide;
  Timer timer;


  public DriveOnRamp( boolean nearSide ) {
    if (nearSide) targetX = 171;
    else targetX = 168;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setFieldRelative(true);
  
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
    if (Math.abs(deltaX) <= 2) {
      timer.start();
     // rampDown = 4;
     // maxSpeed = 0.4;
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
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

   return timer.hasElapsed(4); //
  // return Math.abs(deltaX) < 2;
  }
}
