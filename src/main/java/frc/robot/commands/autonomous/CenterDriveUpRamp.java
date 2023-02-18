// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterDriveUpRamp extends SequentialCommandGroup {
  /** Creates a new CenterDriveUpRamp. */
  public CenterDriveUpRamp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetOdometry(250, -32, 180),
      new DriveToPose(230, -40, 180, 0.3),
      new Wait(1000),
      new DriveOnRamp(true)
    );
  }
}
