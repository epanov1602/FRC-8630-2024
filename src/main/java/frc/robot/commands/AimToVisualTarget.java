// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AimToVisualTarget extends Command {

  private final DriveSubsystem m_drivetrain;
  private final LimelightCamera m_camera;

  // command settings
  private final int m_targetPipelineIndex;
  private final boolean m_seekIfNotFound;
  private final boolean m_stopIfNotFound;

  private static final double TargetXTolerance = 2; // plus minus two degrees is fine
  private static final double SeekingTurnSpeed = 0.15;
  private static final double MaxTurnSpeed = 0.4;

  public AimToVisualTarget(DriveSubsystem drivetrain, LimelightCamera camera, int targetPipelineIndex, boolean seekIfNotFound, boolean stopIfNotFound) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
    m_seekIfNotFound = seekIfNotFound;
    m_stopIfNotFound = stopIfNotFound;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(m_targetPipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = m_camera.getX();
    if (targetX == 0)
       executeTargetNotFound();
    else
       executeTargetFound(targetX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0, 0, 0, Constants.DriveConstants.kFieldRelative, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double targetX = m_camera.getX();

    if (targetX == 0 && m_stopIfNotFound) {
      System.out.print("Must stop aiming because target not found");
      return true;
    }
    else if (targetX == 0) {
      return false;
    }
    else if (-TargetXTolerance < targetX && targetX < TargetXTolerance) {
      System.out.print("target acquired at x=" + targetX);
      return true; // if target is pretty close to center of the screen, we are finished
    } else
      return false; // otherwise, we are not finished yet
  }


  private void executeTargetFound(double targetX) {
      System.out.println("target at x=" + targetX);
      // if X is negative, we want to turn right (and if X is positive, we want to turn left)
      double turnSpeed = -targetX * SeekingTurnSpeed / 15;
      // 30 is the maximum value of X in Limelight: so if X is already closer than 30, we want to be turning slower than SeekingTurnSpeed (to avoid overshooting)
      if (turnSpeed > MaxTurnSpeed)
        turnSpeed = MaxTurnSpeed;
      else if (turnSpeed < -MaxTurnSpeed)
        turnSpeed = -MaxTurnSpeed;
      m_drivetrain.drive(0, 0, turnSpeed, Constants.DriveConstants.kFieldRelative, true);
  }

  private void executeTargetNotFound() {
      // not seeing the target
      if (m_seekIfNotFound) {
        // keep turning until found it, because the user has set m_seekIfNotFound
        m_drivetrain.drive(0, 0, SeekingTurnSpeed, Constants.DriveConstants.kFieldRelative, true);
      } else {
        // do not move when not seeing the target
        m_drivetrain.drive(0, 0, 0, Constants.DriveConstants.kFieldRelative, true);
      }
  }

}
