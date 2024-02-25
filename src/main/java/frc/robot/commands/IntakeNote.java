// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  private Intake m_intake; // TODO: later replace with Manipulator, which would contain an intake

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeNote();
    // TODO: set arm to go down to the pickup angle
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: if the arm is close to pickup angle, drivetrain wiggle right and left and slowly move forward until the note is successfully inside
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    // TODO: drivetrain stop the wiggles
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intake.isNoteInside())
      return true; // if the note is inside, we are finished
    else
      return false; // otherwise, not yet
  }
}
