// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SmartMotionArm;

public class RequestArmAngle extends InstantCommand {
  private final SmartMotionArm m_arm;
  private final double m_angle;

  /** Creates a new RequestArmAngle. */
  public RequestArmAngle(SmartMotionArm arm, double angle) {
    m_arm = arm;
    m_angle = angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleGoal(m_angle);
  }
}
