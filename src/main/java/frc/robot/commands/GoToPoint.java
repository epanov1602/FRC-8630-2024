// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPoint extends Command {
  private final DriveSubsystem m_drivetrain;
  private final double m_targetX, m_targetY;
  private final boolean m_dontSlowDown;

  private Rotation2d m_desiredHeadingWhenDone = null;

  /** Creates a new GoToTarget. */
  public GoToPoint(DriveSubsystem drivetrain, double targetX, double targetY, boolean dontSlowDown) {
    m_drivetrain = drivetrain;
    m_targetX = targetX;
    m_targetY = targetY;
    m_dontSlowDown = dontSlowDown;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredHeadingWhenDone = getDirectionToTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // which direction we want to face/aim?
    Rotation2d aimHere = getDirectionToTarget();
    if (AutoConstants.kOversteerFactor > 0) {
      double oversteerAmount = aimHere.minus(m_desiredHeadingWhenDone).getDegrees() * AutoConstants.kOversteerFactor;
      if (oversteerAmount > AutoConstants.kOversteerCapDegrees)
        oversteerAmount = AutoConstants.kOversteerCapDegrees;
      if (oversteerAmount < -AutoConstants.kOversteerCapDegrees)
        oversteerAmount = -AutoConstants.kOversteerCapDegrees;
      aimHere = aimHere.rotateBy(Rotation2d.fromDegrees(oversteerAmount));
    }
  
    // decide on the turning speed for facing that decided direction asap
    double goToThisDirectionDegrees = aimHere.getDegrees(); /* for example, -88 degrees would mean that the target is almost directly West of us */
    double degreesLeftToTurn = getDegreesLeftToTurn(goToThisDirectionDegrees);
    double turningSpeed = Math.abs(degreesLeftToTurn) * AutoConstants.kPRotation;
    if (turningSpeed > AutoConstants.kMaxTurningSpeed)
      turningSpeed = AutoConstants.kMaxTurningSpeed;
    if (turningSpeed < AutoConstants.kMinTurningSpeed)
      turningSpeed = AutoConstants.kMinTurningSpeed;

    // do we need to know distance to target to slow down before we overshoot it or something?
    double distanceToTarget = getDistanceToTarget();
    double forwardSpeed = AutoConstants.kPTranslation * Math.abs(distanceToTarget);
  
    if (m_dontSlowDown) // if not slowing down at the endpoint, do not slow down even when small distance left to target
      forwardSpeed = AutoConstants.kMaxForwardSpeed;
    if (Math.abs(degreesLeftToTurn) > 90)
      forwardSpeed = 0; // if we have more than 45 degrees left to turn, just keep turning without any forward speed
    if (forwardSpeed > AutoConstants.kMaxForwardSpeed)
      forwardSpeed = Math.min(forwardSpeed, AutoConstants.kMaxForwardSpeed); // do not exceed max forward speed in any situation

    if (degreesLeftToTurn > AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(forwardSpeed, turningSpeed);
    else if (degreesLeftToTurn < -AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(forwardSpeed, -turningSpeed);
    else
      m_drivetrain.arcadeDrive(forwardSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // stop if overshot
    Rotation2d directionToTarget = getDirectionToTarget();
    double cosineToOriginal = directionToTarget.minus(m_desiredHeadingWhenDone).getCos();
    if (cosineToOriginal < 0)
      return true; // done, because overshot

    double distance = getDistanceToTarget();
    if (distance > AutoConstants.kDistanceTolerance)
      return false; // still not close enough

    ChassisSpeeds speed = m_drivetrain.getChassisSpeeds();
    if (Math.abs(speed.vxMetersPerSecond) > AutoConstants.kForwardSpeedTolerance)
        return false; // still moving, not done

    return true;
  }

  // how far are we from the target
  private double getDistanceToTarget() {
    Pose2d currentXY = m_drivetrain.getPose();
    double xDistance = currentXY.getX() - m_targetX;
    double yDistance = currentXY.getY() - m_targetY;
    double totalDistance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);
    return totalDistance;
  }

  private Rotation2d getDirectionToTarget() {
    Pose2d currentXY = m_drivetrain.getPose();
    double xToTarget = m_targetX - currentXY.getX(); // positive X-to-target means we need to be moving further North, negative means we need to move a bit South
    double yToTarget = m_targetY - currentXY.getY(); // positive Y-to-target means we need to be moving further East, negative means further West
    return new Rotation2d(xToTarget, yToTarget);
  }

  private double getDegreesLeftToTurn(double targetDirectionDegrees) {
    // are we heading towards target, or we need to turn?
    double currentHeadingDegrees = m_drivetrain.getPose().getRotation().getDegrees();
    double degreesLeftToTurn = targetDirectionDegrees - currentHeadingDegrees;

    // if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
    while (degreesLeftToTurn > 180)
      degreesLeftToTurn -= 360;

    // if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
    while (degreesLeftToTurn < -180)
      degreesLeftToTurn += 360;

    return degreesLeftToTurn;
  }
}
