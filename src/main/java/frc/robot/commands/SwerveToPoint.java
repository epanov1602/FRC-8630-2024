// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveToPoint extends Command {
  private final DriveSubsystem m_drivetrain;
  private final double m_targetX, m_targetY, m_targetHeadingDegrees;
  private final boolean m_targetHeadingUnspecified;
  private final boolean m_dontSlowDown;

  private double m_initialXDistance = 0;
  private double m_initialYDistance = 0;
  private boolean m_overshot = false;

  /** Creates a new GoToTarget. */
  public SwerveToPoint(DriveSubsystem drivetrain, double targetX, double targetY, boolean dontSlowDown) {
    m_drivetrain = drivetrain;
    m_targetX = targetX;
    m_targetY = targetY;
    m_targetHeadingDegrees = 0;
    m_targetHeadingUnspecified = true;
    m_dontSlowDown = dontSlowDown;
    addRequirements(drivetrain);
  }

  /** Creates a new GoToTarget. */
  public SwerveToPoint(DriveSubsystem drivetrain, double targetX, double targetY, double targetHeadingDegrees, boolean dontSlowDown) {
    m_drivetrain = drivetrain;
    m_targetX = targetX;
    m_targetY = targetY;
    m_targetHeadingDegrees = targetHeadingDegrees;
    m_targetHeadingUnspecified = false;
    m_dontSlowDown = dontSlowDown;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentXY = m_drivetrain.getPose();
    m_initialXDistance = m_targetX - currentXY.getX();
    m_initialYDistance = m_targetY - currentXY.getY();
    m_overshot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentXY = m_drivetrain.getPose();
    double xDistance = m_targetX - currentXY.getX();
    double yDistance = m_targetY - currentXY.getY();
    double totalDistance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);
    if (xDistance * m_initialXDistance + yDistance * m_initialYDistance <= 0)
      m_overshot = true;

    // use the proportional gain to determine desired speed
    double totalSpeed = AutoConstants.kPTranslation * totalDistance;
    if (totalSpeed > AutoConstants.kMaxForwardSpeed || m_dontSlowDown)
      totalSpeed = AutoConstants.kMaxForwardSpeed;

    // distribute that totalSpeed between xSpeed and ySpeed
    double xSpeed = 0, ySpeed = 0;
    if (totalDistance > 0) {
      xSpeed = totalSpeed * xDistance / totalDistance;
      ySpeed = totalSpeed * yDistance / totalDistance;
    }

    // if we should also turn, determine the turning speed
    double turningSpeed = 0;
    if (!m_targetHeadingUnspecified) {
      double degreesLeftToTurn = getDegreesLeftToTurn();
      double absTurningSpeed = Math.abs(degreesLeftToTurn) * Constants.AutoConstants.kPRotation;
      if (absTurningSpeed > Constants.AutoConstants.kMaxTurningSpeed)
        absTurningSpeed = Constants.AutoConstants.kMaxTurningSpeed;
      if (absTurningSpeed < Constants.AutoConstants.kMinTurningSpeed)
        absTurningSpeed = Constants.AutoConstants.kMinTurningSpeed;
      turningSpeed = absTurningSpeed * Math.signum(degreesLeftToTurn); // must be negative if we must turn to negative direction
    }

    m_drivetrain.drive(xSpeed, ySpeed, turningSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_overshot)
      return true;

    double distance = getDistanceToTarget();
    if (distance > AutoConstants.kDistanceTolerance)
      return false; // still not close, finished = false

    ChassisSpeeds speed = m_drivetrain.getChassisSpeeds();
    if (Math.abs(speed.vxMetersPerSecond) + Math.abs(speed.vyMetersPerSecond) > AutoConstants.kForwardSpeedTolerance)
        return false; // still moving, finished = false

    return true;
  }

  // how far are we from the target
  private double getDistanceToTarget() {
    Pose2d currentXY = m_drivetrain.getPose();
    double xDistance = currentXY.getX() - m_targetX;
    double yDistance = currentXY.getY() - m_targetY;
    return Math.sqrt(xDistance * xDistance + yDistance * yDistance);
  }

  private double getDegreesLeftToTurn() {
    double currentHeadingDegrees = m_drivetrain.getPose().getRotation().getDegrees();
    double degreesLeftToTurn = m_targetHeadingDegrees - currentHeadingDegrees;

    // if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
    while (degreesLeftToTurn > 180)
      degreesLeftToTurn -= 360;

    // if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
    while (degreesLeftToTurn < -180)
      degreesLeftToTurn += 360;

    return degreesLeftToTurn;
  }
}
