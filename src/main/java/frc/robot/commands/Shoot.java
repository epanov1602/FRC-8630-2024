// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SmartMotionShooter;

public class Shoot extends Command {
  private static double kMinimumRpm = 100;

  private final SmartMotionShooter m_shooter;
  private final Intake m_intake;
  private final double m_flywheelRpm;

  private double m_feedTime = 0;

  /** Creates a new Shoot. */
  public Shoot(SmartMotionShooter shooter, Intake intake, double flywheelRpm) {
    m_flywheelRpm = Math.max(flywheelRpm, kMinimumRpm);
    // TODO: check agains max rpm
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feedTime = 0; // make a note that we have not fed anything into the flywheel
    m_shooter.setVelocityGoal(m_flywheelRpm); // and start spinning the flywheel
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feedTime == 0 /* if we have not fed anything into flywheel yet */) {
      if (m_shooter.getVelocity() >= 0.85 * m_flywheelRpm /* and if the needed flywheel velocity is almost reached */) {
        m_intake.feedNoteToShooter();
        m_feedTime = Timer.getFPGATimestamp(); /* and note the time when the feeding started */ 
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_feedTime == 0)
      return false; // we have not even started feeding the gamepiece into the shooter
    else if (Timer.getFPGATimestamp() < m_feedTime + 0.5)
      return false; // we have started the feeding, but it was less than a half second ago
    else
      return true; // otherwise, the command is done: the feeding was started and it was started more than 0.5 seconds ago
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: request intake to stop feeding
    m_shooter.setVelocityGoal(0); /* time to stop the flywheel */
    m_intake.stop();
  }

}
