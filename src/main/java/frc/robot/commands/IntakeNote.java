// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SmartMotionArm;

public class IntakeNote extends Command {
  private final Intake m_intake; // TODO: later replace with Manipulator, which would contain an intake
  private final SmartMotionArm m_arm;
  private final DriveSubsystem m_drivetrain;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, SmartMotionArm arm, DriveSubsystem drivetrain) {
    m_intake = intake;
    m_arm = arm;
    m_drivetrain = drivetrain;
    addRequirements(intake);
    addRequirements(arm);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeNote(); // start the intake motor
    m_arm.setAngleGoal(ArmConstants.initialMinAngle); // lower the arm to grab the note
    m_drivetrain.resetWiggleDrive(); // prepare to move forward and wiggle to try our best to pick up the note
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the arm within 3 degrees of pickup angle, start moving towards gamepiece and wiggling right and left

    final double kArmAngleToleranceToPickUp = 3;
    final double kPickupForwardDriveSpeed = 0.2;
    final double kPickupWiggleRotationSpeed = 0.1;
    final double kPickupWiggleIntervalSeconds = 0.5;

    if (m_arm.getAngle() <= ArmConstants.initialMinAngle + kArmAngleToleranceToPickUp)
      m_drivetrain.wiggleDrive(kPickupForwardDriveSpeed, kPickupWiggleRotationSpeed, kPickupWiggleIntervalSeconds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop(); // if the command was interrupted, that motor might still be working and we need to stop it
    m_drivetrain.resetWiggleDrive();
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
