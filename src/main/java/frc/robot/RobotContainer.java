// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AdvanceState;
import frc.robot.commands.GoHome;
import frc.robot.commands.SetFieldRelative;
import frc.robot.commands.SetGamePiece;
import frc.robot.commands.SetScoringLevel;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.autonomous.AutoScore;
import frc.robot.commands.autonomous.AutonomousProfiles;
import frc.robot.commands.autonomous.CenterLeaveCommunity;
import frc.robot.commands.autonomous.InitializedCommandGroup;
import frc.robot.commands.autonomous.PickOneGamePiece;
import frc.robot.commands.autonomous.ScoreTwoGrabThird;
import frc.robot.commands.autonomous.ScoreTwoThenLoopyToRamp;
import frc.robot.commands.autonomous.ScoreTwoThenRamp;
import frc.robot.commands.autonomous.StrafeToVisionTarget;
import frc.robot.subsystems.OperatorStateMachine;
import frc.robot.subsystems.controls.PoseEstimate;
import frc.robot.subsystems.mechanical.HorizontalLiftSubsystem;
import frc.robot.subsystems.mechanical.*;
import frc.robot.subsystems.sensors.Gyro;
import frc.robot.subsystems.sensors.Limelight;
import frc.robot.utilities.state.ScoringState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static Gyro gyro = new Gyro();
  public static SwerveDriveTrain swerveDrive = new SwerveDriveTrain();
  public static Limelight limelight = new Limelight();
  public static PoseEstimate poseEstimator;
  public static HorizontalLiftSubsystem horizontalLiftSubsystem = new HorizontalLiftSubsystem();
  public static VerticalLiftSubsystem verticalLiftSubsystem = new VerticalLiftSubsystem();
  public static Pneumatics pneumatics = new Pneumatics();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static OperatorStateMachine operatorStateMachine = new OperatorStateMachine();
 
  public static CommandXboxController driver = new CommandXboxController(0);
  public static CommandXboxController operator = new CommandXboxController(1);
  public static DataLog dataLog;
  static String alliance = "R";
  static String position = "R";
  SendableChooser<InitializedCommandGroup> chooser = new SendableChooser<>();
  SendableChooser<String> allianceChooser = new SendableChooser<>();
  SendableChooser<String> positionChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new SwerveDriveCommand(driver));
    configureButtonBindings();
    startLogger();
    poseEstimator = new PoseEstimate();
    new AutonomousProfiles();
    //chooser.addOption("Grab Then Ramp", new GrabThenRamp());
    //chooser.addOption("Loopy Path to Charge Station", new LoopyPathToChargeStation());
    chooser.addOption("Just Loopy Path", new LoopyPathToChargeStation());
    chooser.addOption("Score Two Then Loopy To Ramp", new ScoreTwoThenLoopyToRamp());
    chooser.addOption("Score, Pick one, Ramp", new GrabThenRamp());
    chooser.addOption("Score Two Then Center Ramp", new ScoreTwoThenRamp());
    chooser.addOption("Score Second To Third Gamepiece", new ScoreTwoGrabThird());
    chooser.addOption("Drive Over ramp", new CenterLeaveCommunity());
    chooser.addOption("Center, then ramp", new CenterDriveUpRamp());
    //chooser.setDefaultOption("Center, drive over ramp", new CenterLeaveCommunity());
    
    //chooser.addOption("Score Two Game Pieces", new ScoreTwo());
    //chooser.addOption("Score Two Then Ramp", new ScoreTwoThenRamp());
    //chooser.addOption("Score Two Grab Third", new ScoreTwoGrabThird());
    //chooser.addOption("Center Drive Up Ramp", new CenterDriveUpRamp());
    SmartDashboard.putData("Auton Chooser", chooser);
    allianceChooser.setDefaultOption("Red", "R");
    allianceChooser.addOption("Blue", "B");
    SmartDashboard.putData("Alliance Chooser", allianceChooser);
    positionChooser.addOption("Left", "L");
    positionChooser.setDefaultOption("Right", "R");
    SmartDashboard.putData("Position Chooser", positionChooser);

    horizontalLiftSubsystem.resetPosition();
    verticalLiftSubsystem.resetPosition();
  }
  public void stopLogger(){
  }
  public void startLogger() {
    DataLogManager.start();
    dataLog = DataLogManager.getLog();
  }
  public void update(){
    String allianceChoice = allianceChooser.getSelected();
    String positionChoice = positionChooser.getSelected();
    if (allianceChoice != null) alliance = allianceChoice;
    if (positionChoice != null) position = positionChoice;    
  }

  public static String getMatchData() {
    return alliance + position;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}
   */
  private void configureButtonBindings() {
    driver.x().onTrue(new SetFieldRelative(false));
    driver.b().onTrue(new SetFieldRelative(true));
    driver.y().whileTrue(new StrafeToVisionTarget(true));
    driver.a().whileTrue(new StrafeToVisionTarget(false));
    driver.rightBumper().whileTrue(new AutoScore());

    
    operator.rightBumper().onTrue(new AdvanceState());
    operator.leftBumper().onTrue(new GoHome());
    operator.leftTrigger(0.5).onTrue(new SetGamePiece(true));
    operator.rightTrigger(0.5).onTrue(new SetGamePiece(false));
    operator.a().onTrue(new SetScoringLevel(ScoringState.LOW));
    operator.b().onTrue(new SetScoringLevel(ScoringState.MID));
    operator.y().onTrue(new SetScoringLevel(ScoringState.HIGH));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    InitializedCommandGroup command = chooser.getSelected();
    command.initialization();
    System.out.println(command);
    return command;
    // return new AutoPlaceGamePiece(false, OperatorStateMachine.HIGH);
  }

}