// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends InstantCommand {
  DriveSubsystem m_drivetrain;
  Pose2d m_pose;

  public ResetOdometry(DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_pose = null;
    addRequirements(drivetrain);
  }

  public ResetOdometry(DriveSubsystem drivetrain, Pose2d pose) {
    m_drivetrain = drivetrain;
    m_pose = pose;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_pose == null)
      m_drivetrain.resetOdometry();
    else
      m_drivetrain.resetOdometry(m_pose);
  }
}
