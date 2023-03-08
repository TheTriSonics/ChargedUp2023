// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AdvanceState;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetGamePiece;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.SetScoringLevel;
import frc.robot.commands.Wait;
import frc.robot.subsystems.OperatorStateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterDriveUpRamp extends InitializedCommandGroup {
  /** Creates a new CenterDriveUpRamp. */
  public CenterDriveUpRamp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*
    addCommands(
      new SetOdometry(250, -32, 180),
      new DriveToPose(230, -40, 180, 0.3),
      new Wait(1000),
      new DriveOnRamp(true)
    );
    */
  }

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry, target;
    if (matchData.charAt(0) == "R".charAt(0)) {
      odometry = new double[] {250, -60, 180};
      target = new double[] {200, -60, 180};
    } else {
      odometry = new double[] {-250, -60, 0};
      target = new double[] {-200, -60, 0};
    }

    
    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(true, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new DriveToPose(target[0], target[1], target[2], 0.5),
      new DriveOnRamp(true)
    );

  }
}
