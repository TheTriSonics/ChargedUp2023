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
import frc.robot.commands.SetFieldRelative;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.autonomous.AutonomousProfiles;
import frc.robot.commands.autonomous.CenterDriveUpRamp;
import frc.robot.commands.autonomous.DriveOnRamp;
import frc.robot.commands.autonomous.GrabThenRamp;
import frc.robot.commands.autonomous.InitializedCommandGroup;
import frc.robot.commands.autonomous.LoopyPathToChargeStation;
import frc.robot.commands.autonomous.PickOneGamePiece;
import frc.robot.commands.autonomous.ScoreTwo;
import frc.robot.commands.autonomous.ScoreTwoGrabThird;
import frc.robot.commands.autonomous.ScoreTwoThenRamp;
import frc.robot.subsystems.*;
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
    chooser.setDefaultOption("Pick One Game Piece", new PickOneGamePiece());
    chooser.addOption("Pick One Game Piece2", new PickOneGamePiece());
    
    //chooser.addOption("Score Two Game Pieces", new ScoreTwo());
    //chooser.addOption("Score Two Then Ramp", new ScoreTwoThenRamp());
    //chooser.addOption("Score Two Grab Third", new ScoreTwoGrabThird());
    //chooser.addOption("Center Drive Up Ramp", new CenterDriveUpRamp());
    SmartDashboard.putData("Auton Chooser", chooser);
    allianceChooser.setDefaultOption("Red", "R");
    allianceChooser.addOption("Blue", "B");
    SmartDashboard.putData("Alliance Chooser", allianceChooser);
    positionChooser.setDefaultOption("Left", "L");
    positionChooser.addOption("Right", "R");
    SmartDashboard.putData("Position Chooser", positionChooser);
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    InitializedCommandGroup command = chooser.getSelected();
    command.initialization();
    return command;
  }

}