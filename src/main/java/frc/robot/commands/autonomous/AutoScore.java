// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.subsystems.controls.PoseEstimate;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
double zone1Xmin = 80;
double zone1Xmax = 117;
double zone1Ymin = -34;
double zone1Ymax = 94;
Pose2d currentPose;
  /* Use shuffleboard to select a targeted position
   * pass the targeted positions for the waypoints to the path route
   * define "zone 1" boundaries for the field (y: -34 to 94, x: 80 to 117)
   * check to see if in zone 1 and auton was triggered
   * if the check returns true, get current pose and pass it to the path route
   * disable teleop drive controls?
   * check position for possible intake commands
   * run auton command for auto-scoring
   */
  /** Creates a new AutoScore. */
  public AutoScore() {
    /* currentPose = RobotContainer.poseEstimator.getPose();
    if (currentPose.getX()>zone1Xmin && currentPose.getX()<zone1Xmax && currentPose.getY()>zone1Ymin && currentPose.getY()<zone1Ymax){
      AutonomousProfiles.setInitialPose(currentPose.getX(), currentPose.getY());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new  BarCommand());
       addCommands(
       new DriveSwerveProfile(AutonomousProfiles.autoScore.get("RR"), 0.3));
     }*/
     
   }
}
