// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.RotateToHeading;
import frc.robot.commands.SetOdometry;
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
    if (matchData.charAt(0) == "R".charAt(0)) {
      odometry = new double[] {250, -60, 180};
      target = new double[] {80, -60, 180};
      targetHeading = 0;
    } else {
      odometry = new double[] {-250, -60, 0};
      target = new double[] {-80, -60, 0};
      targetHeading = 180;
    }

    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(true, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new DriveToPose(target[0], target[1], target[2], 0.3),
      new RotateToHeading(targetHeading),
      new DriveOnRamp(true)
    );
    
  }
}
