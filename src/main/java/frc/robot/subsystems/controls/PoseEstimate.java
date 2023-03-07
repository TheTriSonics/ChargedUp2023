// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controls;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;

public class PoseEstimate extends SubsystemBase {
  double lastTimeStamp = 0;
  SwerveDrivePoseEstimator poseEstimator;
  DoubleLogEntry llPoseX, llPoseY, llPoseHeading;
  DoubleLogEntry poseX, poseY, poseHeading;
  BooleanLogEntry usedLimeLight;
  /** Creates a new PoseEstimate. */
  public PoseEstimate() {
    llPoseX = new DoubleLogEntry(RobotContainer.dataLog, "llPose X");
    llPoseY = new DoubleLogEntry(RobotContainer.dataLog, "llPose Y");
    llPoseHeading = new DoubleLogEntry(RobotContainer.dataLog, "llPose heading");
    poseX = new DoubleLogEntry(RobotContainer.dataLog, "pose X");
    poseY = new DoubleLogEntry(RobotContainer.dataLog, "pose Y");
    poseHeading = new DoubleLogEntry(RobotContainer.dataLog, "pose heading");
    usedLimeLight = new BooleanLogEntry(RobotContainer.dataLog, "used Lime Light");
    poseEstimator = new SwerveDrivePoseEstimator(
        RobotContainer.swerveDrive.getKinematics(),
        RobotContainer.gyro.getRotation2d(),
        RobotContainer.swerveDrive.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(2,2, Units.degreesToRadians(2)),
        VecBuilder.fill(36, 36, Units.degreesToRadians(30))
        // VecBuilder.fill(10, 20, Units.degreesToRadians(30))
        );
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(double x, double y, double heading) {
    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    RobotContainer.gyro.setInitialHeading(heading);
    poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(), RobotContainer.swerveDrive.getModulePositions(),
        pose);
  }

  boolean aprilTagSeen = true;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(RobotContainer.gyro.getRotation2d(), RobotContainer.swerveDrive.getModulePositions());
    Pose2d pose = getPose();
    Pose2d llPose = RobotContainer.limelight.getPose();
    poseX.append(pose.getX());
    poseY.append(pose.getY());
    poseHeading.append(pose.getRotation().getDegrees());
    if (llPose != null){
      llPoseX.append(llPose.getX());
      llPoseY.append(llPose.getY());
      llPoseHeading.append(llPose.getRotation().getDegrees());
    }else {
      llPoseX.append(Double.NaN);
      llPoseY.append(Double.NaN);
      llPoseHeading.append(Double.NaN);
    }
    double timeStamp = RobotContainer.limelight.getTimeStamp();
    boolean goodTag = false;
    if (llPose != null && timeStamp != lastTimeStamp) {
      if (aprilTagSeen == false) {
        goodTag = true;
        aprilTagSeen = true;
        // RobotContainer.gyro.setInitialHeading(llPose.getRotation().getDegrees());
        poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(),
            RobotContainer.swerveDrive.getModulePositions(), llPose);

      } else {
      //goodTag = true;
        double deltaX = llPose.getX() - pose.getX();
        double deltaY = llPose.getY() - pose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance < 40) { // && llPose.getX() > 160) {
          goodTag = true;
          llPose = new Pose2d(llPose.getX(),llPose.getY(), pose.getRotation());
          poseEstimator.addVisionMeasurement(llPose, RobotContainer.limelight.getLastTimeStamp());
        }
      }
    }
     
    
    usedLimeLight.append(goodTag);
    // poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(),
    // RobotContainer.swerveDrive.getModulePositions(), pose);

    SmartDashboard.putNumber("X Location", pose.getX());
    SmartDashboard.putNumber("Y Location", pose.getY());
    SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees());

  }
}
