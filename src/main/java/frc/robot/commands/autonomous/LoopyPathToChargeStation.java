// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetOdometry;
import frc.robot.subsystems.OperatorStateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoopyPathToChargeStation extends InitializedCommandGroup {
  
  double[][] waypoints = new double[][] {
    {250.0, 30.0},
    {190.87931749063435, 30.397353902458427},
    {150.89655769452479, 25.10596326884294}, // was 122
    {110.71552256808383, 10.218539394196214}, // was 74
    {110.49138476765155, -25.26489623538935}, // was 84
    {120.91379898054784, -40.43045876985129}
  };
  
  double[] headings = new double[] {
      180, 180, 180, 180, 180
  };

  /** Creates a new LoopyPathToChargeStattion. */
  public LoopyPathToChargeStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /* 
    addCommands(
        new SetOdometry(250, 30, 180),
        new DriveSwerveProfile(waypoints, headings, 0.3),
        new DriveOnRamp(false));
    */
  }

  public void initialization() {
    String matchData = RobotContainer.getMatchData();
    double[] odometry = AutonomousProfiles.initialOdometries.get(matchData);
    
    addCommands(
      Commands.parallel(
        new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2])
      ),
      new DriveSwerveProfile(AutonomousProfiles.loopyPath.get(matchData), 0.4),
      new DriveOnRamp(true)
    );
  }
}
