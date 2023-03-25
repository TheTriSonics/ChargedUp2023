// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;

public class DriveOnRamp2 extends CommandBase {
  /** Creates a new DriveOnRampFromNearSide. */
  
  Timer timer;
  double rollOffset = 0;
  double maxSpeed =  0.3;
  boolean timerRunning = false;
  boolean finished = false;
  boolean ascending = false;
  double targetHeading;
  boolean blue;
  boolean back = false;

  public DriveOnRamp2() {
    addRequirements(RobotContainer.swerveDrive);
    timer = new Timer();
    //RobotContainer.poseEstimator.setPose(100, -60, -90);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollOffset = RobotContainer.gyro.getRollAverage();
    timerRunning = false;
    finished = false;
    ascending = false;
    timer = null;
    blue = RobotContainer.getMatchData().startsWith("B", 0);
    //RobotContainer.swerveDrive.setOdometry(new Pose2d(100, -60, new Rotation2d().fromDegrees(-90)));
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
    
    double power = 0.25;
    if (Math.abs(roll) > 12) ascending = true;
    if (ascending) {
      if (timer == null) {
        timer = new Timer();
        timer.start();
      } else {
        if (timer.hasElapsed(1.25)) power = 0.08;
      }
      if (Math.abs(roll) < 10) {
        power = 0;
        back = true;
        timer.reset();
        finished = true;
      }
      if (back && timer.hasElapsed(0.04)) finished = true;
      if (back) power = -0.05;

    }
    if (blue) power *= -1;
    RobotContainer.swerveDrive.drive(power * RobotData.maxSpeed, 0, 0);         
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return finished; 
  }
}
