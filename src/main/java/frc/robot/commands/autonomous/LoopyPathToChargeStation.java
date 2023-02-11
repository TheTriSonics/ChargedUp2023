// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.SetOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoopyPathToChargeStation extends SequentialCommandGroup {
  /*double[][] waypoints = new double[][] {
      { 250.0, 30.0 },
      { 150.0, 30.0 },
      { 120.79310889625387, 18.953645546150465 }, // was 103
      { 120.09483302485617, -45}, // was 103
      { 114.51724723775244, -51}
  }; */
  double[][] waypoints = new double[][] {
    {250.0, 30.0},
    {210.87931749063435, 30.397353902458427},
    {180.89655769452479, 26.10596326884294}, // was 122
    {145.71552256808383, 10.218539394196214}, // was 74
    {150.49138476765155, -10.26489623538935}, // was 84
    {165.91379898054784, -30.43045876985129}
  };
  double[] headings = new double[] {
      180, 180, 180, 180, 180
  };

  /** Creates a new LoopyPathToChargeStattion. */
  public LoopyPathToChargeStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetOdometry(250, 30, 180),
        new DriveSwerveProfile(waypoints, headings, 0.3),
        new DriveOnRamp(false));
  }
}
