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
  double rollOffset = 0;
  double maxSpeed = 0.3; // 0.25;
  double rampDown = 6;
  boolean driveWithDistance = true;
  boolean timerRunning = false;
  boolean finished = false;
  boolean ascending = false;

  public DriveOnRamp( boolean nearSide ) {
    if (nearSide) targetX = 178;
    else {
      targetX = 182;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollOffset = RobotContainer.gyro.getRoll();
    RobotContainer.swerveDrive.setFieldRelative(true);
    timerRunning = false;
    finished = false;
    ascending = false;
  }

  public double normalizeAngle(double x) {
    while (x > 180) x -= 360;
    while (x < -180) x += 360;
    return x;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = RobotContainer.gyro.getRoll() - rollOffset;
    Pose2d pose = RobotContainer.poseEstimator.getPose();
    deltaX = targetX - pose.getX();

    double heading = pose.getRotation().getDegrees();
    double angleError = normalizeAngle(-heading);
    double angleError1 = normalizeAngle(180 - heading);
    if (Math.abs(angleError1) < Math.abs(angleError)) angleError = angleError1;
    double rotSpeed = angleError * RobotData.maxAngularSpeed / 180;

    if (Math.abs(roll) > 10) ascending = true;
    if (Math.abs(roll) < 8 && ascending) finished = true;

    if(deltaX > rampDown) {
      RobotContainer.swerveDrive.drive(maxSpeed * RobotData.maxSpeed, 0, rotSpeed);
      return;
    }
      
    if (Math.abs(deltaX) <= 2 && !timerRunning) {
      timer.start();
      timerRunning = true;
    }
      
    if(Math.abs(deltaX) <= rampDown) {
      double xSpeed = maxSpeed/rampDown * deltaX * RobotData.maxSpeed;
      RobotContainer.swerveDrive.drive(xSpeed, 0, rotSpeed);
      return;      
    } 
   
    RobotContainer.swerveDrive.drive(-maxSpeed * RobotData.maxSpeed, 0, rotSpeed);
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

   return finished || timer.hasElapsed(4); //
  // return Math.abs(deltaX) < 2;
  }
}
