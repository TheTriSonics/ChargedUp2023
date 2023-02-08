// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotData;
import frc.robot.utilities.*;

import javax.xml.xpath.XPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSwerveProfile extends CommandBase implements Runnable {
  /** Creates a new DriveSwerveProfile. */

  // input data:  waypoints -> bezier curves
  BezierCurve[] curves;
  double[] headings;
  int numCurves;
  Point2d initialOffset, deltaInitialOffset;
  Notifier notifier;
  boolean isNotifierRunning = false;


  // length of the trajectory in inches, totalTime to drive it in seconds, and time to decelerate
  double length, totalTime, tDecel;
  
  // some robot data
  double maxSpeed, maxAccel, maxOmega, maxOmegaAccel;

  // states of velocity trapezoidal profile
  
  final int ACCEL = 0;
  final int COAST = 1;
  final int DECEL = 2;
  int state = ACCEL;
  // velocity change in each state
  double[] deltaV = new double[3];
  double[] accel = new double[3];

  // current data
  double deltaT = 0.02;         // 20 ms update
  long lastTime;
  double currentTime = 0;       // current time
  double currentVelocity = 0;   // current velocity
  double currentOmega = 0;      // current max angular velocity
  int currentCurveNumber = 0;   // current index of curve
  double currentS = 0;          // current parameter of current Bezier curve (0 <= s <= 1)
  boolean finished = false;     // finished if we've used up all the curves
  DoubleLogEntry targetX, targetY, poseX, poseY, errorX, errorY, targetHeading, poseHeading,
                 velocityX, velocityY, velocityZ;
  StringLogEntry messages;
  BooleanLogEntry threadRunningLog;


  public DriveSwerveProfile(double[][] waypoints, double[] headings, 
                          double power) {
    // set up input data: headings and bezier curves
    this.headings = headings;
    curves = (new Spline(waypoints)).buildTrajectory();
    numCurves = curves.length;

    // length of the curve in inches
    length = 0;
    for (int i = 0; i < numCurves; i++) length += curves[i].length();

    // set up some robot data
    maxSpeed = RobotData.maxSpeed * power;  // maximum speed we're allowed to drive, we want to cap it
    maxAccel = RobotData.maxAcceleration;   // maximum acceleration, depends on robot
    maxOmega = RobotData.maxAngularSpeed;   // maximum angular speed, depends on robot
    maxOmegaAccel = RobotData.maxAngularAcceleration; // maximum angular acceleration, depends on robot
    double tAccel = maxSpeed/maxAccel;      // time required to accelerate to maximum speed
    double distanceAccel = 0.5*maxAccel * tAccel*tAccel; // distance traveled while accelerating to full speed
    totalTime = (length - 2*distanceAccel)/maxSpeed + 2*tAccel; // total time to drive profile
    tDecel = totalTime - tAccel;            // time we want to start decelerating

    System.out.println("distance: " + length);
    System.out.println("totTime : " + totalTime);
    System.out.println("tDecel  : " + tDecel);
                            
    // changes in velocity in different states
    deltaV[ACCEL] = maxAccel * deltaT;
    deltaV[COAST] = 0;
    deltaV[DECEL] = -maxAccel * deltaT;

    accel[ACCEL] = maxAccel;
    accel[COAST] = 0;
    accel[DECEL] = -maxAccel;

    notifier = new Notifier(this);

    addRequirements(RobotContainer.swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    velocityX = new DoubleLogEntry(RobotContainer.dataLog, "velocityX");
    velocityY = new DoubleLogEntry(RobotContainer.dataLog, "velocityY");
    velocityZ = new DoubleLogEntry(RobotContainer.dataLog, "velocityZ");
    targetX = new DoubleLogEntry(RobotContainer.dataLog, "target X");
    targetY = new DoubleLogEntry(RobotContainer.dataLog, "target Y");
    poseX = new DoubleLogEntry(RobotContainer.dataLog, "pose X");
    poseY = new DoubleLogEntry(RobotContainer.dataLog, "pose Y");
    errorX = new DoubleLogEntry(RobotContainer.dataLog, "error X");
    errorY = new DoubleLogEntry(RobotContainer.dataLog, "error Y");
    targetHeading = new DoubleLogEntry(RobotContainer.dataLog, "target heading");
    poseHeading = new DoubleLogEntry(RobotContainer.dataLog, "pose heading");
    messages = new StringLogEntry(RobotContainer.dataLog, "messages");
    threadRunningLog = new BooleanLogEntry(RobotContainer.dataLog, "is notifier stopped");

    Pose2d currentPose = RobotContainer.swerveDrive.getOdometry().getPoseMeters();
    Point2d currentPosition = new Point2d(currentPose.getX(), currentPose.getY());
    Point2d targetPosition = curves[0].getPosition(0);
    initialOffset = Math2d.diff2d(targetPosition, currentPosition);
    deltaInitialOffset = Math2d.smult(0.01, initialOffset);
    lastTime = System.currentTimeMillis();

    isNotifierRunning = true;
    notifier.startPeriodic(0.010);

    messages.append("Initializing Drive Swerve Profile 4");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public boolean isNotifierFinished(){
    return !isNotifierRunning;
    
  }

  public void stopNotifier() {
    isNotifierRunning = false;
  }

  public void run() {
    long time = System.currentTimeMillis();
    if (time == lastTime) return;
    deltaT = (time - lastTime)/1000.0;
    //System.out.println(time + " " + lastTime + " " + deltaT);
    lastTime = time;
    // update velocity depending on the state
    if (currentTime > tDecel) state = DECEL;
    currentVelocity += accel[state] * deltaT;
    if (currentVelocity > maxSpeed) {
      currentVelocity = maxSpeed;
      state = COAST;
    }

    if (currentVelocity < 0) {
      stopNotifier();
      messages.append("profile finished: v < 0");
      threadRunningLog.append(isNotifierFinished());
      System.out.println("FINSIHED: " + currentTime);
      return;
    }

    // Find velocity feedforward term from the profile
    
    Point2d currentTangent = curves[currentCurveNumber].getTangent(currentS);
    Point2d feedForward = Math2d.smult(currentVelocity/currentTangent.length(), currentTangent);

    // Find the error term from where we are and where we should be

    Point2d currentPosition = curves[currentCurveNumber].getPosition(currentS);
    if (initialOffset.length() > 1) {
      currentPosition = Math2d.diff2d(currentPosition, initialOffset);
      initialOffset = Math2d.diff2d(initialOffset, deltaInitialOffset);
    }
    Pose2d currentPose = RobotContainer.swerveDrive.getOdometry().getPoseMeters(); 

   
    Point2d deltaPosition = new Point2d(currentPosition.x - currentPose.getX(), currentPosition.y - currentPose.getY());
    double kP = 0.15;
    double errorSpeed = kP * deltaPosition.length()/deltaT;
    if (errorSpeed > 0.5*currentVelocity) errorSpeed = 0.5*currentVelocity;
    Point2d errorVelocity = Math2d.smult(errorSpeed/deltaPosition.length(), deltaPosition);
    Point2d speedPoint2d = Math2d.sum2d(feedForward, errorVelocity);
    
    
    velocityX.append(RobotContainer.gyro.getVelocityX());
    velocityY.append(RobotContainer.gyro.getVelocityY());
    velocityZ.append(RobotContainer.gyro.getVelocityZ());
    targetX.append(currentPosition.x);
    targetY.append(currentPosition.y);
    poseX.append(currentPose.getX());
    poseY.append(currentPose.getY());
    errorX.append(deltaPosition.x);
    errorY.append(deltaPosition.y);


    double xSpeed = speedPoint2d.x;
    double ySpeed = speedPoint2d.y;
    
    // Prepare to update new parameter on curve
    double deltaS = currentVelocity * deltaT / currentTangent.length();
    double newS = currentS + deltaS;
    //System.out.println(currentCurveNumber);
    if (newS > 1) { // we are going to the next curve
      // this is a little hacky, math could be improved, but probably not a big deal
      newS -= 1;
      currentCurveNumber += 1;
      if (currentCurveNumber >= numCurves) {  // we've used up the curves so we're done
        System.out.println("FINSIHED: " + currentTime);
        finished = true;
        currentCurveNumber -= 1;
        newS = 1;
        currentOmega = maxOmega/2.0;
        stopNotifier();
        messages.append("profile finished");
        threadRunningLog.append(isNotifierFinished());
        return;
      }
    }

    messages.append("profile running");
    threadRunningLog.append(isNotifierFinished());
    
    // now deal with the heading of the robot
    double deltaAngle = headings[currentCurveNumber] - currentPose.getRotation().getDegrees();
    while (deltaAngle < -180) deltaAngle += 360;
    while (deltaAngle > 180) deltaAngle -= 360;
    currentOmega += maxOmegaAccel * deltaT;
    if (currentOmega > maxOmega) currentOmega = maxOmega;
    double rotSpeed = deltaAngle/180 * currentOmega;

    targetHeading.append(headings[currentCurveNumber]);
    poseHeading.append(currentPose.getRotation().getDegrees());
    

    // finally, tell the drive train what to do
    //RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rotSpeed);
    //System.out.println(xSpeed + " " + ySpeed);  
    RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rotSpeed);  

    // update current parameter on current curve and current time
    currentS = newS;
    currentTime += deltaT;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopDriveMotor();
    notifier.stop();
    System.out.println("FINISHED: " + RobotContainer.swerveDrive.getOdometry().getPoseMeters());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isNotifierFinished();// || currentTime > totalTime;
  }
}