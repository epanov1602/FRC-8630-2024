// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.Constants.OdometryConstants;
import frc.robot.commands.AimToDirection;
import frc.robot.commands.DropArmForPickup;
import frc.robot.commands.EjectFromShooter;
import frc.robot.commands.FollowVisualTarget;
import frc.robot.commands.SwerveToPoint;
import frc.robot.commands.SwerveTrajectoryToPoint;
import frc.robot.commands.SwitchVisualTarget;
import frc.robot.commands.GoToPoint;
import frc.robot.commands.MockPickupCommand;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.RequestArmAngle;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.subsystems.SmartMotionArm;
import frc.robot.subsystems.SmartMotionShooter;
import frc.robot.commands.EjectNote;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // all robot's subsystems
  private static DriveSubsystem m_drivetrain = new DriveSubsystem();
  private static LimelightCamera m_pickupCamera = new LimelightCamera(CameraConstants.kPickupCameraName);
  private static LimelightCamera m_aimingCamera = new LimelightCamera(CameraConstants.kAimingCameraName);
  private SmartMotionArm m_arm = new SmartMotionArm();
  private SmartMotionShooter m_shooter = new SmartMotionShooter();
  private Intake m_intake = new Intake(); // TODO: once arm and shooter are integrated, maybe make a composite manipulator subsystem out of them?

  // all teleop controllers
  private CommandXboxController m_driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_manipulatorJoystick = new CommandXboxController(OIConstants.kManipulatorController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default teleop commands
    if (Constants.DriveConstants.kCopterJoystickLayout)
      m_drivetrain.setDefaultCommand(new RunCommand(this::copterJoystickDrive, m_drivetrain));
    else
      m_drivetrain.setDefaultCommand(new RunCommand(this::tankJoystickDrive, m_drivetrain));
  
    // the default command for the aiming camera is to change the LEDs depending on whether the gamepiece is inside of intake or not
    // (default command only runs if no other command needs to run)
    m_aimingCamera.setDefaultCommand(m_aimingCamera.run(() -> {
      if (m_intake.isNoteInside())
         m_aimingCamera.setLightOn();
      else
         m_aimingCamera.setLightOff();
    }));

    m_pickupCamera.setPipeline(CameraConstants.kNotePipelineIndex);
    m_aimingCamera.setPipeline(CameraConstants.kSpeakerPipelineIndex);
  }

  private void tankJoystickDrive() {
    // tank layoyt: left stick for movement, right stick for rotation
    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_driverJoystick.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getRightX(), OIConstants.kDriveDeadband),
        Constants.DriveConstants.kFieldRelative,
        true);
  }

  private void copterJoystickDrive() {
    // if keeping high pressure on the throttle stick, not field-relative anymore (for manual aiming)
    double slowDownFactor = 1.0;
    boolean fieldRelative = Constants.DriveConstants.kFieldRelative;
    if (Math.abs(m_driverJoystick.getLeftY()) > 0.5) {
      fieldRelative = false;
      slowDownFactor = 0.75;
    }

    // copter layoyt otherwise: right stick for movement, left stick for rotation
    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_driverJoystick.getRightY() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getRightX() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getLeftX() * slowDownFactor, OIConstants.kDriveDeadband),
        fieldRelative,
        true);
  }

  private void configureButtonBindings() {
    // let's put all the button bindings here, to keep them in one place

    // POV up: raise, aim and shoot at angle 37, and rpm 5700 (it does not have to be 5700 since the target is nearby, but this is for later)
    Command raiseAndShoot = makeRaiseAndShootCommand(37, 5700);
    m_driverJoystick.povUp().onTrue(raiseAndShoot);

    // POV left: pick up the piece using arm and drivetrain (to automatically wiggle-drive towards it, maximizing the chances of pickup)
    Command pickUpWithDriving = makeApproachNoteCommand(true, 80); // raise arm to 37 degrees after pickup
    m_driverJoystick.povLeft().whileTrue(pickUpWithDriving);

    // POV down: pick up the piece using just arm (but not automatically driving towards it)
    Command pickUpWithoutDriving = makePickupNoteCommand(true, 80); // raise arm to 30 degrees after pickup
    m_driverJoystick.povDown().whileTrue(pickUpWithoutDriving);

    // POV right: eject the note reliably
    Command ejectNote = makeConsistentEjectNoteCommand();
    m_driverJoystick.povRight().onTrue(ejectNote);

    // raw movements of the manipulator, in order to troubleshoot it
    m_driverJoystick.y().onTrue(m_arm.runOnce(() -> m_arm.setAngleGoal(80)));
    m_driverJoystick.a().onTrue(m_arm.runOnce(() -> m_arm.setAngleGoal(ArmConstants.initialMinAngle)));
    m_driverJoystick.x().onTrue(m_shooter.runOnce(() -> m_shooter.setVelocityGoal(2000)));
    m_driverJoystick.b().onTrue(m_shooter.runOnce(() -> m_shooter.setVelocityGoal(0)));
  }

  private Command makeRaiseAndShootCommand(double aimArmAngle, double shootingFlywheelRpm) {
    Command dropArm = new RaiseArm(m_arm, aimArmAngle - 15);
    Command raiseArm = new RaiseArm(m_arm, aimArmAngle, "raiseAngle");

    Command shoot = new Shoot(m_shooter, m_intake, shootingFlywheelRpm);

    Command result = new SequentialCommandGroup(dropArm, raiseArm, shoot);
    return result;
  }

  private Command makeEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp); // is 94 a good angle to eject the note into the amp reliably? 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp); // is 80 a good angle for pushing the note in
    Command result = new SequentialCommandGroup(raiseArm, ejectAndPush);
    return result;
  }

  private Command makeConsistentEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp); // is 94 a good angle to eject the note into the amp reliably? 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp); // is 80 a good angle for pushing the note in
    Command raiseEjectAndPush = new SequentialCommandGroup(raiseArm, ejectAndPush);

    // to do it more reliably, ensure that the bumper has fully approached the amp *and* ensure consistent starting angle 
    Command dropToLowerAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp - 5); // 5 degrees lower
    Command raiseToPushAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp); // good starting angle
    Command ejectRoutine = new SequentialCommandGroup(dropToLowerAngle, raiseToPushAngle, raiseEjectAndPush);

    // one way to ensure that bumper is touching the amp wall is to be driving towards amp all this time
    Command beDriving = m_drivetrain.run(() -> m_drivetrain.arcadeDrive(0.2, 0));

    // the result is "be driving until the eject routine is completed"
    return ejectRoutine.deadlineWith(beDriving);
  }

  private Command makePickupNoteCommand(boolean driveTowards, double armAngleAfterPickup) {  
    // 1. take the note
    Command grabNote;
    if (driveTowards == true)
      grabNote = new IntakeNote(m_intake, m_arm, m_drivetrain, armAngleAfterPickup);
    else /* if driveTowards==false, we are not supposed to drive towards the note, so do not let the command use m_drivetrain */
      grabNote = new IntakeNote(m_intake, m_arm, null, armAngleAfterPickup);

    // 2. after the note is in, it might be blocking the shooter from spinning: move it back by a few inches
    Command unblockShooter = new EjectNote(m_intake, null, 0.05, 0).withTimeout(0.1); // speed=5%, and add timeout=0.2

    // 0 + 1 + 2
    Command result = new SequentialCommandGroup(grabNote, unblockShooter);
    return result;
  }

  private Command makeApproachNoteCommand(boolean driveTowards, double armAngleAfterPickup) {
    var whenToStop = new FollowVisualTarget.WhenToFinish(-16, 0, 0, false);
    FollowVisualTarget follow = new FollowVisualTarget(
      m_drivetrain, m_pickupCamera, 9, 0.05, 0.5,
      CameraConstants.kPickupCameraImageRotation, whenToStop);
    
    var andThen = makePickupNoteCommand(driveTowards, armAngleAfterPickup);
    return new SequentialCommandGroup(follow, andThen);
  }

  private boolean operatorUsingSticks() {
    double totalInput = Math.abs(m_driverJoystick.getLeftX()) + Math.abs(m_driverJoystick.getLeftY())
        + Math.abs(m_driverJoystick.getRightX()) + Math.abs(m_driverJoystick.getRightY());
    return totalInput > 0.1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // the default auto command is to raise the arm to 80 degree angle
    return new RaiseArm(m_arm, 80);
  }

  public static void testInit() {
    // run testInit() on each subsystem.
    m_drivetrain.testInit();
  }

  public static void testPeriodic() {
    // run testPeriodic() on each subsystem
    m_drivetrain.testPeriodic();
  }
}
