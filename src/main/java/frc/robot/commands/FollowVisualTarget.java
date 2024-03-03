// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowVisualTarget extends Command {
  public static double kDirectionToleranceDegrees = AutoConstants.kDirectionToleranceDegrees;

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
    public final double m_slowdownRadiusPercentage;

    public WhenToFinish(double minTargetY, double maxTargetY, double maxTargetSize, boolean finishIfNotMoving, double slowdownRadiusPercentage) {
      m_minTargetY = minTargetY;
      m_maxTargetY = maxTargetY;
      m_maxTargetSize = maxTargetSize;
      m_finishIfNotMoving = finishIfNotMoving;
      m_slowdownRadiusPercentage = slowdownRadiusPercentage;
    } 

    public WhenToFinish(double minTargetY, double maxTargetY, double maxTargetSize, boolean finishIfNotMoving) {
      this(minTargetY, maxTargetY, maxTargetSize, finishIfNotMoving, 0.30); // default slowdown radius = 30% of angle
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
    if (Math.abs(seekingTurnSpeed) > AutoConstants.kMaxTurningSpeedForVisualTargets)
      seekingTurnSpeed = Math.signum(seekingTurnSpeed) * AutoConstants.kMaxTurningSpeedForVisualTargets;
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
    m_camera.setComputerVisionMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = m_camera.getX(m_imageRotation);
    if (targetX == 0) {
      // target not found: stop or seek?
      if (m_seekingTurnSpeed == 0 && m_whenToFinish.m_finishIfNotMoving) {
        if (m_camera.getPercentageOfTimeTargetDetected() < 0.25) {
          System.out.println("stopping because target is not seen anymore");
          m_timeToStop = true;
        } else {
          System.out.println("target is not seen now, but we keep watching");
          double seekSpeed = m_lastSeenTargetX > 0 ? AutoConstants.kMinTurningSpeed : -AutoConstants.kMinTurningSpeed;
          m_drivetrain.arcadeDrive(0, seekSpeed, false);
        }
        return;
      }
      System.out.println("target is not seen now, but we keep looking for it");
      double seekingDirection = 1;
      if (m_lastSeenTargetX > 0)
        seekingDirection = -1;
      m_drivetrain.arcadeDrive(0, m_seekingTurnSpeed * seekingDirection, false);
    } else {
      // target found: turn towards it
      double degreesLeftToTurn = -targetX;
      double turningSpeed = Math.abs(degreesLeftToTurn) * Constants.AutoConstants.kPRotation;
      m_lastSeenTargetX = targetX;

      // should we be turning towards it? or just moving straight?
      if (turningSpeed < Constants.AutoConstants.kMinTurningSpeed)
        turningSpeed = Constants.AutoConstants.kMinTurningSpeed;
      if (turningSpeed > Constants.AutoConstants.kMaxTurningSpeedForVisualTargets && m_seekingTurnSpeed == 0) // only applies when seeking is disabled
        turningSpeed = Constants.AutoConstants.kMaxTurningSpeedForVisualTargets;
      if (turningSpeed > Constants.AutoConstants.kMaxTurningSpeed)
        turningSpeed = Constants.AutoConstants.kMaxTurningSpeed;
      if (Math.abs(degreesLeftToTurn) < kDirectionToleranceDegrees)
        turningSpeed = 0;
      else if (degreesLeftToTurn < 0)
        turningSpeed = -turningSpeed;

      // should we be moving fast?
      double forwardSpeed = m_approachSpeed;
      if (forwardSpeed != 0) {
        // reduce the forward speed if we also need to turn (otherwise the motor that needs to spin faster might not be able to do it)
        double reduction = Math.abs(turningSpeed / forwardSpeed);
        if (reduction > 0.5)
          reduction = 0.5; // do not make wheels spin the opposite way though
        forwardSpeed = forwardSpeed * (1 - reduction);
      }

      // are we finished?
      double percentDistanceToTarget = 1.0;
      if (m_whenToFinish.m_maxTargetSize > 0) {
        double size = m_camera.getA();
        percentDistanceToTarget = Math.abs(size - m_whenToFinish.m_maxTargetSize) / m_whenToFinish.m_maxTargetSize; 
        if (size > m_whenToFinish.m_maxTargetSize) {
          System.out.println("stopping because reached maxTargetSize: " + m_camera.getA());
          m_timeToStop = true;
        }
      }
      if (m_whenToFinish.m_maxTargetY != 0) {
        double targetY = m_camera.getY();
        percentDistanceToTarget = Math.abs((targetY - m_whenToFinish.m_maxTargetY) / m_whenToFinish.m_maxTargetY);
        if (targetY > m_whenToFinish.m_maxTargetY) {
          System.out.println("stopping because reached maxTargetY: " + m_camera.getY());
          m_timeToStop = true;
        }
      }
      if (m_whenToFinish.m_minTargetY != 0) {
        double targetY = m_camera.getY();
        percentDistanceToTarget = Math.abs((targetY - m_whenToFinish.m_minTargetY) / m_whenToFinish.m_minTargetY);
        if (targetY < m_whenToFinish.m_minTargetY) {
          System.out.println("stopping because reached minTargetY: " + m_camera.getY());
          m_timeToStop = true;
        }
      }
      if (forwardSpeed == 0 && turningSpeed == 0 && m_whenToFinish.m_finishIfNotMoving) {
        System.out.println("stopping because we see the target and there are no further moves to make");
        m_timeToStop = true;
      }

      // restrictions on forward speed
      double noOvershootSpeed = Math.sqrt(percentDistanceToTarget);
      if (Math.abs(forwardSpeed) > noOvershootSpeed)
        forwardSpeed = noOvershootSpeed * Math.signum(m_approachSpeed);
      if (Math.abs(forwardSpeed) < AutoConstants.kMinTurningSpeed)
        forwardSpeed = AutoConstants.kMinForwardSpeed * Math.signum(m_approachSpeed);
      if (Math.abs(degreesLeftToTurn) > 60)
        forwardSpeed = 0;

      SmartDashboard.putNumber("rotSpeed", turningSpeed);
      SmartDashboard.putNumber("fwdSpeed", forwardSpeed);
      m_drivetrain.drive(forwardSpeed, 0, turningSpeed, false, false);
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
    if (m_timeToStop)
      System.out.println("stopping for some reason");
    return m_timeToStop;
  }
}
