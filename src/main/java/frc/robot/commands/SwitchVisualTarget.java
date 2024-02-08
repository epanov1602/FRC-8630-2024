// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimelightCamera;

public class SwitchVisualTarget extends InstantCommand {
  private final LimelightCamera m_camera;
  private final int m_pipelineIndex;

  public SwitchVisualTarget(LimelightCamera camera, int pipelineIndex) {
    m_camera = camera;
    m_pipelineIndex = pipelineIndex;
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(m_pipelineIndex);
  }
}
