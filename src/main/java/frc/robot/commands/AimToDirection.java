// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AimToDirection extends Command {
  private DriveSubsystem m_drivetrain;
  private Translation2d m_targetDirectionPoint;
  private double m_targetDirectionDegrees;

  public AimToDirection(DriveSubsystem drivetrain, double targetDirectionDegrees) {
    m_drivetrain = drivetrain;
    m_targetDirectionDegrees = targetDirectionDegrees;
    m_targetDirectionPoint = null;
    addRequirements(drivetrain);
  }

  public AimToDirection(DriveSubsystem drivetrain, Translation2d targetFieldPoint) {
    m_drivetrain = drivetrain;
    m_targetDirectionPoint = targetFieldPoint;
    m_targetDirectionDegrees = 0;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_targetDirectionPoint != null) {
      Translation2d fromHereToTargret = m_targetDirectionPoint.minus(m_drivetrain.getPose().getTranslation());
      m_targetDirectionDegrees = fromHereToTargret.getAngle().getDegrees();
    }
    SmartDashboard.putNumber("aimToDirectionDegrees", m_targetDirectionDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degreesLeftToTurn = getDegreesLeftToTurn();
    double turningSpeed = Math.abs(degreesLeftToTurn) * Constants.AutoConstants.kPRotation;
    if (turningSpeed > Constants.AutoConstants.kMaxTurningSpeed)
      turningSpeed = Constants.AutoConstants.kMaxTurningSpeed;
    if (turningSpeed < Constants.AutoConstants.kMinTurningSpeed)
      turningSpeed = Constants.AutoConstants.kMinTurningSpeed;

    if (degreesLeftToTurn > Constants.AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(0, turningSpeed);
    else if (degreesLeftToTurn < -Constants.AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(0, -turningSpeed);
    else
      m_drivetrain.arcadeDrive(0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // are we facing good angle?
    double degreesLeftToTurn = getDegreesLeftToTurn();
    if (Math.abs(degreesLeftToTurn) > Constants.AutoConstants.kDirectionToleranceDegrees) {
      return false;
    }

    // we are at good angle, but is the chassis stopped? (i.e. won't overshoot the turn)
    ChassisSpeeds chassisSpeeds = m_drivetrain.getChassisSpeeds();
    double turningSpeed = chassisSpeeds.omegaRadiansPerSecond * (180.0 / Math.PI);
    if (Math.abs(turningSpeed) > Constants.AutoConstants.kTurningSpeedToleranceDegreesPerSecond) {
      return false;
    }

    // if we are here, we are aimed and the chassis is almost not moving 
    return true;
  }

  private double getDegreesLeftToTurn() {
    // are we heading towards target, or we need to turn?
    double currentHeadingDegrees = m_drivetrain.getPose().getRotation().getDegrees();
    double degreesLeftToTurn = m_targetDirectionDegrees - currentHeadingDegrees;

    // if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
    while (degreesLeftToTurn > 180)
      degreesLeftToTurn -= 360;

    // if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
    while (degreesLeftToTurn < -180)
      degreesLeftToTurn += 360;

    return degreesLeftToTurn;
  }
}
