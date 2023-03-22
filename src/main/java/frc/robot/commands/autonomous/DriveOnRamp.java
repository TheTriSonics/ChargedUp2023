// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double maxSpeed =  0.3;
  double rampDown = 6;
  boolean timerRunning = false;
  boolean finished = false;
  boolean ascending = false;
  double targetHeading;

  public DriveOnRamp( boolean nearSide ) {
    String matchData = RobotContainer.getMatchData();
    boolean left = (matchData == "RL" || matchData == "BL");
    if (nearSide) targetX = 180;
    else {
      targetX = 210;
      if (left) targetX = 120;
    }
    addRequirements(RobotContainer.swerveDrive);
    timer = new Timer();
    
  }

  public DriveOnRamp(boolean nearSide, double rampTarget) {
    this(nearSide);
    targetX = rampTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollOffset = RobotContainer.gyro.getRollAverage();
    timerRunning = false;
    finished = false;
    ascending = false;
    String matchData = RobotContainer.getMatchData();
    //System.out.println("Orignal match data value: " + matchData);
    if (matchData.startsWith("B", 0)) {
      targetX *= -1;
      targetHeading = 180;
    } else {
      targetHeading = 0;
    }
    boolean left = matchData.charAt(1) == 'L';
    //System.out.println("left value: " + left);
    if (nearSide) {
      //System.out.println("on nearside");
      targetX = 180;
    } else {
      //System.out.println("NOT on nearside");
      targetX = 207;
      if (left) {
        //System.out.println("Still knew to go left");
        targetX = 178;
      }
      //targetX = 173;
    }
  }

  public double normalizeAngle(double x) {
    while (x > 180) x -= 360;
    while (x < -180) x += 360;
    return x;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = RobotContainer.gyro.getRollAverage() - rollOffset;
    //SmartDashboard.putNumber("Roll on ramp", roll);
    //System.out.println(roll);
    Pose2d pose = RobotContainer.poseEstimator.getPose();
    deltaX = targetX - pose.getX();

    double heading = pose.getRotation().getDegrees();
    double angleError = normalizeAngle(-heading);
    double angleError1 = normalizeAngle(180 - heading);
    if (Math.abs(angleError1) < Math.abs(angleError)) angleError = angleError1;
    double rotSpeed = -0.2*angleError * RobotData.maxAngularSpeed / 180;
    rotSpeed = 0;

    if (Math.abs(roll) > 12) ascending = true;
    if (Math.abs(roll) < 10 && ascending) {
      end(false);
      finished = true;
      return;
    }

    double speed;
    if(Math.abs(deltaX) > rampDown) {
      speed = maxSpeed * RobotData.maxSpeed;
      if (deltaX > 0) RobotContainer.swerveDrive.drive(maxSpeed * RobotData.maxSpeed, 0, rotSpeed);
      else {
        speed *= -1;
        RobotContainer.swerveDrive.drive(-maxSpeed * RobotData.maxSpeed, 0, rotSpeed);
      }
    } else {
      double xSpeed = deltaX/rampDown * maxSpeed * RobotData.maxSpeed;
      speed = xSpeed;
      RobotContainer.swerveDrive.drive(xSpeed, 0, rotSpeed);     
    } 
    // System.out.println(RobotContainer.getMatchData() + " " + pose.getX() + " " + deltaX + " " + speed/RobotData.maxSpeed);
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
