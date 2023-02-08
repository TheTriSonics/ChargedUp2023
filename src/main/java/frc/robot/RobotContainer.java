// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sql.rowset.serial.SerialArray;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetFieldRelative;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.autonomous.LoopyPathToChargeStation;
import frc.robot.commands.autonomous.PickOneGamePiece;
//import frc.robot.commands.*;
//import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;
//import frc.robot.utilities.ShotData;
//import frc.robot.utilities.XboxTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static Gyro gyro = new Gyro();
  public static SwerveDriveTrain swerveDrive = new SwerveDriveTrain();
  public static Limelight limelight = new Limelight();
  //public static Odometry odometry = new Odometry();
  public static PoseEstimate poseEstimator = new PoseEstimate();

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);
  public static DataLog dataLog;

  //Command m_autoCommand = new ThreeBallAuto(); //DriveToPose(100, 20, 90, 0.4);
  SendableChooser chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new SwerveDriveCommand(driver));
    // Configure the button bindings
    //swerveDrive.setDefaultCommand(new SwerveDriveCommand(driver));

    configureButtonBindings();

    //CameraServer.startAutomaticCapture();
    
  }

  public void startLogger() {
    DataLogManager.start();
    dataLog = DataLogManager.getLog();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}
   */
  private void configureButtonBindings() {
    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverLBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    JoystickButton driverStartButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    JoystickButton driverBackButton = new JoystickButton(driver, XboxController.Button.kBack.value);

    driverX.whenPressed(new SetFieldRelative(false));
    driverB.whenPressed(new SetFieldRelative(true));
    
    /*
    driverA.whenPressed(new DriveToCargo(false));
    */

    //driverY.whenPressed(new IncrementShooterSpeed(500));
    //driverA.whenPressed(new IncrementShooterSpeed(-500));

    //driverA.whenPressed(new SetShooter(11000));
    //driverY.whenPressed(new SetShooter(0));
    // driverX.whenPressed(new TogglePhotoEye(true));
    // driverB.whenPressed(new TogglePhotoEye(false));

    JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    JoystickButton operatorLBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    JoystickButton operatorRBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    
    // operatorLBumper.whenPressed(new SetIntake(true));
    // operatorRBumper.whenPressed(new SetIntake(false));
    //operatorX.whenPressed(new SetShot(ShotData.FEET12));
    // operatorA.whenPressed(new SetShot(ShotData.FEET10));
    //operatorY.whenPressed(new SetShot(ShotData.FEET16));
    //operatorB.whenPressed(new SetShot(ShotData.BUMPER));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return new PickOneGamePiece();
  }
  
}
