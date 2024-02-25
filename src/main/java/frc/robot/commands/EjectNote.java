// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class EjectNote extends Command {
  private final Intake m_intake; // TODO: later replace with Manipulator, which would contain an intake
  private final double m_speed;

  private double m_startTime = 0;

  /** Creates a new DiscardNote. */
  public EjectNote(Intake intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.ejectNote(m_speed);
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() < m_startTime + 1.0)
      return false; // have been running for less than one second, the note might not be fully out yet
    if (m_intake.isNoteInside())
      return false; // looks like the note is still inside
    return true; // otherwise we are done
  }
}
