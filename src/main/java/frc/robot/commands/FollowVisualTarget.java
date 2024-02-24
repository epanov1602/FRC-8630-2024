// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowVisualTarget extends Command {

  private final DriveSubsystem m_drivetrain;
  private final LimelightCamera m_camera;
  private final Rotation2d m_imageRotation;

  // command settings
  private final int m_targetPipelineIndex;
  private final double m_seekingTurnSpeed, m_approachSpeed;

  // finishing criteria
  public static class WhenToFinish {
    public final double m_minTargetY;
    public final double m_maxTargetY;
    public final double m_maxTargetSize;
    public final boolean m_finishIfNotMoving;

    public WhenToFinish(double minTargetY, double maxTargetY, double maxTargetSize, boolean finishIfNotMoving) {
      m_minTargetY = minTargetY;
      m_maxTargetY = maxTargetY;
      m_maxTargetSize = maxTargetSize;
      m_finishIfNotMoving = finishIfNotMoving;
    } 
  }
  private final WhenToFinish m_whenToFinish;

  private double m_lastSeenTargetX = 0;
  private int m_lastPipelineIndex = -1;
  private boolean m_timeToStop = false;
  private boolean m_endedWithTarget = false;

  /* target was visible at the time the command ended (whether it finished or got interrupted) */
  public boolean getEndedWithTarget() { return m_endedWithTarget; }

  /* target was visible at the time the command ended *and* it finished (did not get interrupted) */
  public boolean getFinishedWithTarget() { return m_endedWithTarget && m_timeToStop; }

  public FollowVisualTarget(
    DriveSubsystem drivetrain, 
    LimelightCamera camera,
    int targetPipelineIndex,
    double seekingTurnSpeed,
    double approachSpeed,
    Rotation2d rotateImageToLeft,
    WhenToFinish whenToStop) 
  {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
    m_seekingTurnSpeed = seekingTurnSpeed;
    m_approachSpeed = approachSpeed;
    m_imageRotation = rotateImageToLeft;
    m_whenToFinish = whenToStop;
    addRequirements(drivetrain);
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeToStop = false;
    m_endedWithTarget = false;
    m_lastSeenTargetX = 0;
    m_lastPipelineIndex = m_camera.getPipeline();
    m_camera.setPipeline(m_targetPipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = m_camera.getX(m_imageRotation);
    if (targetX == 0) {
      // target not found: stop or seek?
      if (m_seekingTurnSpeed == 0 && m_whenToFinish.m_finishIfNotMoving) {
        m_timeToStop = true;
        return;
      }
      double seekingDirection = 1;
      if (m_lastSeenTargetX > 0)
        seekingDirection = -1;
      m_drivetrain.drive(0, 0, m_seekingTurnSpeed * seekingDirection, false, true);
    } else {
      // target found: turn towards it
      double degreesLeftToTurn = -targetX;
      double turningSpeed = Math.abs(degreesLeftToTurn) * Constants.AutoConstants.kPRotation;
      m_lastSeenTargetX = targetX;

      // should we be moving towards it?
      double forwardSpeed = m_approachSpeed;
      if (turningSpeed > Constants.AutoConstants.kMaxTurningSpeed) {
        turningSpeed = Constants.AutoConstants.kMaxTurningSpeed;
        forwardSpeed = 0; // we are not aimed too well
      }
      if (turningSpeed < Constants.AutoConstants.kMinTurningSpeed)
        turningSpeed = Constants.AutoConstants.kMinTurningSpeed;
      if (degreesLeftToTurn < 0)
        turningSpeed = -turningSpeed;
      if (Math.abs(degreesLeftToTurn) < Constants.AutoConstants.kDirectionToleranceDegrees)
        turningSpeed = 0;

      // are we finished?
      if (m_whenToFinish.m_maxTargetSize > 0 && m_camera.getA() > m_whenToFinish.m_maxTargetSize)
        m_timeToStop = true;
      if (m_whenToFinish.m_maxTargetY != 0 && m_camera.getY() > m_whenToFinish.m_maxTargetY)
        m_timeToStop = true;
      if (m_whenToFinish.m_minTargetY != 0 && m_camera.getY() < m_whenToFinish.m_minTargetY)
        m_timeToStop = true;
      if (forwardSpeed == 0 && turningSpeed == 0 && m_whenToFinish.m_finishIfNotMoving)
        m_timeToStop = true;

      m_drivetrain.drive(forwardSpeed, 0, turningSpeed, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.arcadeDrive(0, 0);
      m_endedWithTarget = m_camera.getPercentageOfTimeTargetDetected() > 0.5;
      m_camera.setPipeline(m_lastPipelineIndex);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timeToStop;
  }
}
