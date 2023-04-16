// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AdvanceState;
import frc.robot.commands.DriveSwerveProfile;
import frc.robot.commands.GoHome;
import frc.robot.commands.InScoringPosition;
import frc.robot.commands.SetGamePiece;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.SetOperatorState;
import frc.robot.commands.SetScoringLevel;
import frc.robot.commands.SetUseAprilTags;
import frc.robot.commands.Wait;
import frc.robot.subsystems.OperatorStateMachine;
import frc.robot.subsystems.mechanical.VerticalLiftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePiece extends InitializedCommandGroup {
  
  /** Creates a new PickOneGamePiece. */
  public ThreePiece() {}
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  public void initialization() {
    Map<String, double[]> initialOdometries = new HashMap<String, double[]>();
    initialOdometries.put("RR", new double[] {250, 45, 0});
    initialOdometries.put("RL", new double[] {250, -141, 0});
    initialOdometries.put("BR", new double[] {-250, -141, 180});
    initialOdometries.put("BL", new double[] {-250, 40, 180});
    double[] redTarget1 = new double[] {64, 35, 0};
    double[] blueTarget1 = new double[] {-62, 30, 180};
    double[] target1;
    String matchData = RobotContainer.getMatchData();
    if (matchData.startsWith("R")) {
      target1 = redTarget1;
      matchData = "RR";
    }
    else {
      matchData = "BL";
      target1 = blueTarget1;
    }
  
    double[] odometry = initialOdometries.get(matchData);
    System.out.println("INITIALIZING THREE PIECE");
    addCommands(
      Commands.parallel(
        //new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH), 
        new SetOdometry(odometry[0], odometry[1], odometry[2]),
        new SetUseAprilTags(false),
        new SetOperatorState(OperatorStateMachine.ENGAGEGAMEPIECE)
      ),
    
      Commands.parallel(
    //     //new DriveSwerveProfile(AutonomousProfiles.straightToFirstGamePiece.get(matchData), 0.4, true),// 0.6), 
        Commands.sequence(
          new Wait(700),
          new DriveToPose(target1[0], target1[1], target1[2], 0.55)
        ),
      Commands.parallel(
        Commands.sequence(new Wait(1500), new SetOperatorState(OperatorStateMachine.GAMEPIECEPREP)),
          new SetGamePiece(false),
          new SetScoringLevel(OperatorStateMachine.LOW)
        )
      ),
    
    //     //new WaitForPhotoEye(),
      new AdvanceState(),
    //   //new SetGamePiece(true),
      Commands.parallel(
        new DriveSwerveProfile(AutonomousProfiles.firstGamePieceToSecondPlacement.get(matchData), 0.4), 
        Commands.sequence(
          new Wait(250),
          new AdvanceState(),
          new Wait(50)
        )
      ),
      Commands.parallel(
    //     // new AdvanceState(),
        new InScoringPosition()
      ),
      new AdvanceState(),
      new Wait(300),
      Commands.parallel(
        new DriveSwerveProfile(AutonomousProfiles.driveToSecondGamePiece.get(matchData), 0.4, true), //0.6));
        Commands.sequence(
          new Wait(500),
          new AdvanceState(),
          new SetGamePiece(false),
          new Wait(1000),
          new AdvanceState()
        )
      ),
      new AdvanceState(),
      Commands.parallel(
        Commands.sequence(
          new Wait(250),
          new AdvanceState(),
          new Wait(50),
          new SetVerticalLift(VerticalLiftSubsystem.RIGHTCONE)
        ),
        new DriveSwerveProfile(AutonomousProfiles.thirdPiece.get(matchData), 0.45)
      ),
      new AdvanceState()
    );
  }
}
