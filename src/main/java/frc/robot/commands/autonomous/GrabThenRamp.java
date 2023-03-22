// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.subsystems.OperatorStateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabThenRamp extends InitializedCommandGroup {
  /** Creates a new GrabThenRamp. */
  double[] redTarget = new double[] {100, -60, -90};
  double[] blueTarget = new double[] {-100, -45, 90};
  double[] target;
  double targetHeading;
  public GrabThenRamp() {

  }

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    boolean red = matchData.startsWith("R", 0);
    

    boolean left = matchData.equals("RL") || matchData.equals("BL");
    if (left) {
      redTarget[0] = 80;
      blueTarget[0] = -80;
    }

    if (red) {
      target = redTarget;
      targetHeading = -90;
    }
    else {
      target = blueTarget;
      targetHeading = 90;
    }
    
    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new ParallelCommandGroup(
        new DriveSwerveProfile(AutonomousProfiles.driveToFirstGamePiece.get(matchData), 0.3, true),// 0.6), 
        Commands.parallel(
          Commands.sequence(new Wait(1500), new AdvanceState())),
          new SetGamePiece(true),
          new SetScoringLevel(OperatorStateMachine.HIGH)
        ),
        new AdvanceState(),
        //new SetUseAprilTags(true),
        new Wait(400),
        new DriveToPose(target[0], target[1], target[2], 0.50),
        new DriveOnRamp2()
    );
  }
}
