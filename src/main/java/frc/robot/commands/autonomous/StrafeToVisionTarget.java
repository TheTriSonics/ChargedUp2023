// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotData;
import frc.robot.Constants.RobotConstants;

public class StrafeToVisionTarget extends CommandBase {
  boolean left;
  boolean finished = false;
  /** Creates a new StrafeToVisionTarget. */
  public StrafeToVisionTarget(boolean left) {
    this.left = left;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] target = RobotContainer.limelight.getRetroTarget();
    double heading = RobotContainer.gyro.getYaw();
    double headingError = heading - 0;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    double rotationSpeed = headingError/180 * RobotData.maxAngularSpeed;
    //System.out.println(target[0] + " " + target[1]);
    if (target == null) {
      double ySpeed = 0.3 * RobotData.maxSpeed;
      if (!left) ySpeed = -0.3 * RobotData.maxSpeed;
      RobotContainer.swerveDrive.drive(0, ySpeed, rotationSpeed);
      return;
    }
    //System.out.println(target[0] + " " + target[1]);
    double targetX = 202;
    if (target[1] < 150) targetX = 189;
    finished = Math.abs(targetX - target[0]) < 5;
    double ySpeed = -0.1/50*(targetX - target[0]) * RobotData.maxSpeed;
    RobotContainer.swerveDrive.drive(0, ySpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
