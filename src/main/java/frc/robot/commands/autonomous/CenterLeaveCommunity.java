// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.RotateToHeading;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.SetUseAprilTags;
import frc.robot.commands.Wait;
import frc.robot.subsystems.OperatorStateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterLeaveCommunity extends InitializedCommandGroup {
  /** Creates a new CenterLeaveCommunity. */
  public CenterLeaveCommunity() {}
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry, target;
    double targetHeading;
    double rampTarget;
    if (matchData.startsWith("R")) {
      odometry = new double[] {250, -80, 180};
      target = new double[] {80, -60, 180};
      rampTarget = 182;
      targetHeading = -90;
    } else {
      odometry = new double[] {-250, -80, 0};
      target = new double[] {-80, -100, 0};
      rampTarget = 182;
      targetHeading = 90;
    }

    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
       // new SetUseAprilTags(true)
      ),
      new DriveToPose(target[0], target[1], target[2], 0.3),
      new RotateToHeading(targetHeading),
      //new DriveOnRamp(true, rampTarget)
      new DriveOnRamp2()
    );
    
  }
}
