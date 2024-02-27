// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SmartMotionArm;
import frc.robot.subsystems.SmartMotionShooter;

public class EjectFromShooter extends Command {
  private final SmartMotionShooter m_shooter;
  private final SmartMotionArm m_arm;
  private final Intake m_intake;
  private double m_startTime = 0; 

  /** Creates a new EjectFromShooter. */
  public EjectFromShooter(SmartMotionShooter shooter, Intake intake, SmartMotionArm arm) {
    m_shooter = shooter;
    m_intake = intake;
    m_arm = arm;
    addRequirements(intake);
    addRequirements(shooter);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_arm.setAngleGoal(80); // highest angle to reach
    m_intake.feedNoteToShooter();
    m_shooter.setVelocityGoal(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + 1.0; // give it one second to run
  }
}
