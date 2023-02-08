// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveTrain;

public class SwerveDriveCommand extends CommandBase {
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  CommandXboxController controller;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(CommandXboxController controller) {
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  public double getSquareInput(double x) {
    if (x >= 0) {
      return x * x;
    } else {
      return -(x * x);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = getSquareInput(MathUtil.applyDeadband(controller.getLeftY(), 0.05));
    double yInput = getSquareInput(MathUtil.applyDeadband(controller.getLeftX(), 0.05));
    double rotInput = getSquareInput(MathUtil.applyDeadband(controller.getRightX(), 0.05));
    final var xSpeed = -m_xspeedLimiter.calculate(xInput)
        * SwerveDriveTrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(yInput)
        * SwerveDriveTrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -0.5 * m_rotLimiter.calculate(rotInput)
        * SwerveDriveTrain.kMaxAngularSpeed;

    // System.out.println(xSpeed + " " + ySpeed + " " + rot + " " +
    // System.currentTimeMillis());
    double scale = -1;
    if (RobotContainer.swerveDrive.isFieldRelative())
      scale = 1;
    RobotContainer.swerveDrive.drive(scale * xSpeed, scale * ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
