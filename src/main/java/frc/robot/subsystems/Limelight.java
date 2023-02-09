// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  final double INCHESPERMETER = 39.36;
  double tx, ty, tv;
  Pose2d pose;
  NetworkTableEntry jsonDumpNetworkTableEntry;
  NetworkTable networkTable;
  double lastTimeStampSeconds = 0;

  /** Creates a new Limelight. */
  public Limelight() {
    jsonDumpNetworkTableEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json");
  }

  public boolean isTargetValid() {
    return tv > .5;
  }

  public double getTargetX() {
    return tx;
  }

  public double getTargetY() {
    return ty;
  }

  public double getRobotX() {
    // double[] pose =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new
    // double[]{});
    return pose.getX();
  }

  public double getRobotY() {
    // double[] pose =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new
    // double[]{});
    return pose.getY();
  }

  public Rotation2d getRobotRotation() {
    // double[] pose =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new
    // double[]{});
    return pose.getRotation();
  }

  public Pose2d getPose() {
    return pose;
  }

  int timesSeen = 0;
  int requiredTimesSeen = 4;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
     * tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").
     * getDouble(0);
     * SmartDashboard.putNumber("tx", tx);
     * ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").
     * getDouble(0);
     * SmartDashboard.putNumber("ty", ty);
     * tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").
     * getDouble(0);
     * SmartDashboard.putNumber("tv", tv);
     */
    double[] newPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
        .getDoubleArray(new double[] { 1234 });
    if (newPose.length < 6 || (newPose[0] == 0.0 && newPose[1] == 0)) {
      pose = null;
      timesSeen = 0;
      return;
    }
    timesSeen++;
    if (timesSeen < requiredTimesSeen)
      return;
    pose = new Pose2d(newPose[0] * INCHESPERMETER, newPose[1] * INCHESPERMETER, Rotation2d.fromDegrees(newPose[5]));

    String jsonDump = jsonDumpNetworkTableEntry.getString("{}");

    double currentTimeStampSeconds = lastTimeStampSeconds;
    // Attempts to get the time stamp for when the robot pose was calculated
    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      double tsValue = jsonNodeData.path("Results").path("ts").asDouble();
      SmartDashboard.putNumber("tsValue", tsValue);
      if (tsValue != 0) {
        // Converts from milleseconds to seconds
        currentTimeStampSeconds = tsValue / 1000;
      }
      lastTimeStampSeconds = currentTimeStampSeconds;

    } catch (JsonProcessingException e) {
      SmartDashboard.putString("Json Parsing Error", "oops");
    }
  }

  public double getLastTimeStamp() {
    return lastTimeStampSeconds;
  }

}
