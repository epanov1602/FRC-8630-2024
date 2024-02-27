// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SmartMotionArm;

public class DropArmForPickup extends Command {
  private static final double kAngleTolerance = 4; // degrees

  private final SmartMotionArm m_arm;

  /** Creates a new DropArmForPickup. */
  public DropArmForPickup(SmartMotionArm arm) {
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleGoal(15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if dropped pretty close to zero degrees, we are finished dropping the arm for pickup (and can proceed to intake)
    return Math.abs(m_arm.getAngle() - 0) < kAngleTolerance; 
  }
}
