// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SmartMotionArm;

import java.util.function.Supplier;

public class RaiseArm extends Command {
  private static final double kAngleTolerance = 8; // degrees

  private final String m_ntTargetAngleName;
  private final Supplier<Double> m_targetAngleSupplier;

  private final SmartMotionArm m_arm;
  private double m_targetAngle;

  public RaiseArm(SmartMotionArm arm, double targetAngle) {
    this(arm, targetAngle, null, null);
  }

  /** Creates a new RaiseArm. */
  public RaiseArm(SmartMotionArm arm, double targetAngle, Supplier<Double> targetAngleSupplier, String networkTablesAngleName) {
    m_arm = arm;
    m_targetAngle = targetAngle;
    m_targetAngleSupplier = targetAngleSupplier;
    m_ntTargetAngleName = networkTablesAngleName;
  
    addRequirements(arm);

    // if target angle was specified too low or too high, bring it into limits
    if (m_targetAngle > ArmConstants.initialMaxAngle)
      m_targetAngle = ArmConstants.initialMaxAngle;
    if (m_targetAngle < ArmConstants.initialMinAngle)
      m_targetAngle = ArmConstants.initialMinAngle;
  
    if (m_ntTargetAngleName != null)
      SmartDashboard.setDefaultNumber(m_ntTargetAngleName, targetAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // -- if target angle comes from somewhere, get it from there
    if (m_ntTargetAngleName != null)
      setTargetAngle(SmartDashboard.getNumber(m_ntTargetAngleName, m_targetAngle));
    else if (m_targetAngleSupplier != null)
      setTargetAngle(m_targetAngleSupplier.get());
    // -- then, set it on the arm
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
    if (m_arm.getAngleVelocity() < 0)
      return false; // if the arm is on its way down, it will overshoot and later come back; not finished yet
    double distanceToTarget = Math.abs(m_arm.getAngle() - m_targetAngle);
    if (distanceToTarget > kAngleTolerance)
      return false;
    // otherwise, we arrived
    return true;
  }

  private void setTargetAngle(double targetAngle) {
    if (targetAngle > ArmConstants.initialMaxAngle)
      targetAngle = ArmConstants.initialMaxAngle;
    if (targetAngle < ArmConstants.initialMinAngle)
      targetAngle = ArmConstants.initialMinAngle;
    m_targetAngle = targetAngle;
  }
}
