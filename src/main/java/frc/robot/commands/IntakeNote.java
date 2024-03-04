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
  private final double m_armAngleAfterIntaking;

  /** Creates a new IntakeNote
   * @param armAngleAfterIntaking if nonzero, to which angle to raise the arm after intaking
   * @param drivertrain if not null, which drivetrain to use to approach the gamepiece
   */
  public IntakeNote(Intake intake, SmartMotionArm arm, DriveSubsystem drivetrain, double armAngleAfterIntaking) {
    m_intake = intake;
    m_arm = arm;
    m_drivetrain = drivetrain;
    m_armAngleAfterIntaking = armAngleAfterIntaking;
    addRequirements(intake);
    if (arm != null)
      addRequirements(arm);
    if (drivetrain != null)
      addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeNote(); // start the intake motor
    if (m_drivetrain != null)
      m_drivetrain.resetWiggleDrive(); // prepare to move forward and wiggle to try our best to pick up the note
    if (m_arm != null)
      m_arm.setAngleGoal(ArmConstants.initialMinAngle); // lower the arm to grab the note
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the arm within 3 degrees of pickup angle, start moving towards gamepiece and wiggling right and left

    final double kArmAngleToleranceToPickUp = 10;

    // these three constants were calibrated by Brian, hope they work for everyone
    final double kPickupForwardDriveSpeed = 0.75; // any reason to not use max speed here?
    final double kPickupWiggleRotationSpeed = 0.2;
    final double kPickupWiggleIntervalSeconds = 0.5;

    if (m_drivetrain != null) {
      // wait until the arm is low enough
      if (m_arm == null || m_arm.getAngle() <= ArmConstants.initialMinAngle + kArmAngleToleranceToPickUp)
        m_drivetrain.wiggleDrive(kPickupForwardDriveSpeed, kPickupWiggleRotationSpeed, kPickupWiggleIntervalSeconds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop(); // if the command was interrupted, that motor might still be working and we need to stop it
    if (m_arm != null && m_armAngleAfterIntaking != 0 && m_intake.isNoteInside())
      m_arm.setAngleGoal(m_armAngleAfterIntaking);
    if (m_drivetrain != null)
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
