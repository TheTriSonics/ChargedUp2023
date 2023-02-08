// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Odometry extends SubsystemBase {
  private Pose2d pose;
  final double INCHESPERMETER = 39.36;

  /** Creates a new Odometry. */
  public Odometry() {}

  public Pose2d getPose(){
    return pose;
  }

  @Override
  public void periodic() {
    /*
    Pose2d newPose = RobotContainer.limelight.getPose();
    if (newPose == null) {
      RobotContainer.swerveDrive.updateOdometry();
      pose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    } else {
  
      pose = newPose;
      RobotContainer.swerveDrive.setOdometry(pose);
    }
    */
    /*SmartDashboard.putNumber("X Location", pose.getX());
    SmartDashboard.putNumber("Y Location", pose.getY());
    SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees()); */
  
  }
}
