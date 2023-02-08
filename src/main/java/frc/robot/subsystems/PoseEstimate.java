// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PoseEstimate extends SubsystemBase {

  SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new PoseEstimate. */
  public PoseEstimate() {



    poseEstimator = new SwerveDrivePoseEstimator(
      RobotContainer.swerveDrive.getKinematics(),
      RobotContainer.gyro.getRotation2d(), 
      RobotContainer.swerveDrive.getModulePositions(), 
      new Pose2d());
  }

  public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
  }

  boolean aprilTagSeen = false; 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(RobotContainer.gyro.getRotation2d(), RobotContainer.swerveDrive.getModulePositions());

    Pose2d llPose = RobotContainer.limelight.getPose();

    if(llPose != null) {
    
      if(aprilTagSeen == false) {
       
        aprilTagSeen = true;
        //RobotContainer.gyro.setInitialHeading(llPose.getRotation().getDegrees());
        poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(), RobotContainer.swerveDrive.getModulePositions(), llPose);
      
      } else {
        poseEstimator.addVisionMeasurement(llPose, RobotContainer.limelight.getLastTimeStamp());
      }
   }

    Pose2d pose = getPose();
    //poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(), RobotContainer.swerveDrive.getModulePositions(), pose);

    SmartDashboard.putNumber("X Location", pose.getX());
    SmartDashboard.putNumber("Y Location", pose.getY());
    SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees());
  

  }
}
