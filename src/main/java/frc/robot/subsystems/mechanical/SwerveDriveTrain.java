// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanical;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase implements Runnable {
  Notifier notifier;

  // TODO: Robot constants will need to be tuned
  public static final double kMaxSpeed = 15.6 * 12; // inches per second 4.758; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1 rotation per second

  // TODO: Location of wheels from center of robot need to be defined
  private final static double halfWheelBase = 27.750/2.0;  // 23.251 / 2.0;
  private final static double halfTrackWidth = 21.250/2.0;  // 22.25 / 2.0;
  private final Translation2d m_frontLeftLocation = new Translation2d(halfWheelBase, halfTrackWidth);
  private final Translation2d m_frontRightLocation = new Translation2d(halfWheelBase, -halfTrackWidth);
  private final Translation2d m_backLeftLocation = new Translation2d(-halfWheelBase, halfTrackWidth);
  private final Translation2d m_backRightLocation = new Translation2d(-halfWheelBase, -halfWheelBase);

  // TOD: CAN Bus IDs need to be defined to match physical robot
  // ... and be placed in a nice central location like 'RobotMap'
  /*
   * private final SwerveModule m_frontLeft = new SwerveModule(10, 20, 0, 2528,
   * "Front Left", false);
   * private final SwerveModule m_backLeft = new SwerveModule(11, 21, 1, 1642,
   * "Back Left", false);
   * private final SwerveModule m_backRight = new SwerveModule(12, 22, 2, 730,
   * "Back Right", false);
   * private final SwerveModule m_frontRight = new SwerveModule(13, 23, 3, 3907,
   * "Front Right", false);
   */
  private final SwerveModule m_backRight = new SwerveModule(10, 20, 0, 3482, "Front Left", false);
  private final SwerveModule m_frontRight = new SwerveModule(11, 21, 1, 325, "Back Left", false);
  private final SwerveModule m_frontLeft = new SwerveModule(12, 22, 2, 3467, "Back Right", false);
  private final SwerveModule m_backLeft = new SwerveModule(13, 23, 3, 3066, "Front Right", false);

  boolean fieldRelative = true;
  boolean driveAligned = false;
  double goalX = 27 * 12;
  double goalY = 27 * 6;
  double kP = 0.015;

  private final SwerveModulePosition[] defaultPos = new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
  };
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
      RobotContainer.gyro.getRotation2d(), defaultPos);

  public SwerveDriveTrain() {
    RobotContainer.gyro.reset();
    notifier = new Notifier(this);
    notifier.startPeriodic(0.01);
    m_odometry.resetPosition(new Rotation2d(), defaultPos, new Pose2d());
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void setDriveAligned(boolean aligned) {
    driveAligned = aligned;
  }

  public void toggleDriveAligned() {
    driveAligned = !driveAligned;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot) {
    if (driveAligned) {
      if (RobotContainer.limelight.isTargetValid()) {
        double x = -RobotContainer.limelight.getTargetX();
        rot = kP * x * kMaxAngularSpeed;
      } else {
        Pose2d currentPose = getOdometry().getPoseMeters();
        double deltaX = goalX - currentPose.getX();
        double deltaY = goalY - currentPose.getY();
        double targetAngle = Math.atan2(deltaY, deltaX) * 180 / Math.PI;
        double errorAngle = targetAngle - currentPose.getRotation().getDegrees();
        while (errorAngle < -180)
          errorAngle += 360;
        while (errorAngle > 180)
          errorAngle += 360;
        rot = errorAngle / 180 * kMaxAngularSpeed;

      }

    }
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RobotContainer.gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void resetOdometry() {
    m_odometry.resetPosition(new Rotation2d(), defaultPos, new Pose2d());
  }

  public void setOdometry(Pose2d pose) {
    setOdometry(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public void setOdometry(double x, double y, double heading) {
    RobotContainer.gyro.setInitialHeading(heading);
    m_odometry.resetPosition(RobotContainer.gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        },
        new Pose2d(x, y, new Rotation2d(heading * Math.PI / 180)));
  }

  public void setBrakeMode() {
    m_frontLeft.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_frontRight.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_backLeft.getDriveMotor().setNeutralMode(NeutralMode.Brake);
    m_backRight.getDriveMotor().setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    m_frontLeft.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_frontRight.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_backLeft.getDriveMotor().setNeutralMode(NeutralMode.Coast);
    m_backRight.getDriveMotor().setNeutralMode(NeutralMode.Coast);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        RobotContainer.gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        });
  }

  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState(),
    };
  }

  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public void resetTurnEncoders() {
    m_frontLeft.resetTurnEncoder();
    m_backLeft.resetTurnEncoder();
    m_backRight.resetTurnEncoder();
    m_frontRight.resetTurnEncoder();
  }

  public void stopDriveMotor() {
    m_backRight.stopDriveMotor();
    m_backLeft.stopDriveMotor();
    m_frontRight.stopDriveMotor();
    m_frontLeft.stopDriveMotor();
  }

  public void run() {
    updateOdometry();
  }

  public SwerveModule[] getModules() {
    SwerveModule[] modules = new SwerveModule[] { m_backRight, m_backLeft, m_frontRight, m_frontLeft };
    return modules;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Back Right", m_backRight.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Back Left", m_backLeft.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Front Right", m_frontRight.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Front Left", m_frontLeft.getAbsoluteTurnPosition());
    Pose2d pose = getOdometry().getPoseMeters();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Rotation", pose.getRotation().getDegrees());

    // updateOdometry();
    // System.out.println(fieldRelative);
  }
}
