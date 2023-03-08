// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

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
  public GrabThenRamp() {
    double[][] waypoints = new double[][] {
      { 250.0, 30.0 },
      { 190.87931749063435, 30.397353902458427 },
      { 150.5344885237755, 30.10596326884294 },
      { 67.03448798270918, 20.390731496573693 }
    };
    double[] headings = new double[] {
      100, 60, 0
    };
    double[][]waypoints2 = new double[][] {
      {72.62069495389075, 26.821195041112187},
      {85.88793651044695, -13.231784205965681},
      {107.5344885237755, -51.854299908505055}
    };
    double[] headings2 = new double[] {
      150, 180
    };
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    
    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new ParallelCommandGroup(
        new DriveSwerveProfile(AutonomousProfiles.driveToFirstGamePiece.get(matchData), 0.3),// 0.6), 
        Commands.parallel(
          Commands.sequence(new Wait(1500), new AdvanceState())),
          new SetGamePiece(false),
          new SetScoringLevel(OperatorStateMachine.HIGH)
        ),
        new AdvanceState(),
        new Wait(200),
        new DriveToPose(100, -60, 180, 0.45),
        new DriveOnRamp(false)
    );
  }
}
