// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SmartMotionArm;

public class RaiseArm extends Command {
  private static final double kAngleTolerance = 8; // degrees

  private final SmartMotionArm m_arm;
  private double m_targetAngle;

  /** Creates a new RaiseArm. */
  public RaiseArm(SmartMotionArm arm, double targetAngle) {
    m_arm = arm;
    m_targetAngle = targetAngle;
  
    // if target angle was specified too low or too high, bring it into limits
    if (m_targetAngle > ArmConstants.initialMaxAngle)
      m_targetAngle = ArmConstants.initialMaxAngle;
    if (m_targetAngle < ArmConstants.initialMinAngle)
      m_targetAngle = ArmConstants.initialMinAngle;
  
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleGoal(m_targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do, the arm is working to get to the angle
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceToTarget = Math.abs(m_arm.getAngle() - m_targetAngle);
    if (distanceToTarget < kAngleTolerance)
      return true;
    else
      return false;
  }
}
