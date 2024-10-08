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
import frc.robot.commands.EjectFromShooter;
import frc.robot.commands.FollowVisualTarget;
import frc.robot.commands.SwerveToPoint;
import frc.robot.commands.SwerveTrajectoryToPoint;
import frc.robot.commands.SwitchVisualTarget;
import frc.robot.commands.GoToPoint;
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
import frc.utils.CubicSpline;

import static frc.robot.Constants.ArmConstants.initialMinAngle;

import java.lang.reflect.Field;
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

    Command setDefaultCameraPipelines = m_pickupCamera.runOnce(this::setDefaultCameraPipelines);

    // Configure default teleop commands
    if (Constants.OIConstants.kCopterJoystickLayout)
      m_drivetrain.setDefaultCommand(setDefaultCameraPipelines.andThen(new RunCommand(this::copterJoystickDrive, m_drivetrain)));
    else
      m_drivetrain.setDefaultCommand(setDefaultCameraPipelines.andThen(new RunCommand(this::tankJoystickDrive, m_drivetrain)));
  
    // the default command for the aiming camera is to change the LEDs depending on whether we are close enough to fire
    // (default command only runs if no other command needs to run)
    m_aimingCamera.setDefaultCommand(m_aimingCamera.run(() -> {
      if (m_aimingCamera.getY() > -1.4) // if target Y angle altitude is >0, we are close enough to fire from this distance ~successfully
         m_aimingCamera.setLightOn();
      else
         m_aimingCamera.setLightOff();
    }));

    setDefaultCameraPipelines();
  }

  private void setDefaultCameraPipelines() {
    m_pickupCamera.setPipeline(CameraConstants.kNotePipelineIndex);
    m_aimingCamera.setPipeline(CameraConstants.kSpeakerPipelineIndex);
  }

  private void tankJoystickDrive() {
    // tank layout: left stick for movement, right stick for rotation
    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_driverJoystick.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getRightX(), OIConstants.kDriveDeadband),
        Constants.OIConstants.kFieldRelative,
        true);
  }

  private void copterJoystickDrive() {
    // if keeping high pressure on the throttle stick, not field-relative anymore (for manual aiming)
    double slowDownFactor = 1.0;
    boolean fieldRelative = Constants.OIConstants.kFieldRelative;
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
    // driver can:
    // - X brake for blocking,
    // - lower arm to dive under the chain
    // - raise arm back
    m_driverJoystick.x().whileTrue(m_drivetrain.run(m_drivetrain::setX));
    m_driverJoystick.a().onTrue(new RaiseArm(m_arm, ArmConstants.initialMinAngle, 0));
    m_driverJoystick.y().onTrue(new RaiseArm(m_arm, ArmConstants.kArmAgleToSaveEnergy, 0));
  
    // operator can: do all else with the arm
    var joystick = m_manipulatorJoystick;
    if (!Constants.OIConstants.useTwoJoysticks)
      joystick = m_driverJoystick; // if not using two joysticks, use driver joystick for both driving and arm

    Command approachAndShoot = makeApproachAndShootCommand(31.5, 2850, "armShootAngle"); // can make it "armShootAngle"
    joystick.rightBumper().whileTrue(approachAndShoot);

    Command brakeAndShoot = makeBrakeAndShootCommand();
    joystick.leftBumper().whileTrue(brakeAndShoot);

    // POV up: just shoot assuming we are up close, don't even look at the camera
    Command raiseAndShoot = makeRaiseAndShootCommand(31.5, 5700, "armShootAngle"); // can make it "armShootAngle" (another option: 37, 5700)
    joystick.povUp().onTrue(raiseAndShoot);

    // POV left: pick up using camera
    Command approachAndPickup = makeApproachNoteCommand(80); // raise arm to 80 degrees after pickup, to save energy
    joystick.povLeft().whileTrue(approachAndPickup);

    // POV down: pick up without using camera (by just driving towards the note until it is picked up)
    Command pickUpWithDrivingTowards = makePickupNoteCommand(true, 80); // raise arm to 80 degrees after pickup, to save energy
    joystick.povDown().whileTrue(pickUpWithDrivingTowards);

    // POV right: eject the note reliably
    Command ejectNote = makeConsistentEjectNoteCommand();
    joystick.povRight().onTrue(ejectNote);

    // raw movements of the manipulator, in order to troubleshoot it (only if we have the second joystick)
    if (joystick != m_driverJoystick)
    {
      joystick.y().onTrue(m_arm.runOnce(() -> m_arm.setAngleGoal(80)));
      joystick.a().onTrue(m_arm.runOnce(() -> m_arm.setAngleGoal(ArmConstants.initialMinAngle)));
      joystick.x().onTrue(m_shooter.runOnce(() -> m_shooter.setVelocityGoal(2850)));
      joystick.b().onTrue(m_shooter.runOnce(() -> m_shooter.setVelocityGoal(0)));
    }
  }

  private Command makeRaiseAndShootCommand(double aimArmAngle, double shootingFlywheelRpm, String setAngleFromSmartDashboardKey) {
    Command dropArm = new RaiseArm(m_arm, aimArmAngle - 15, 0); // TODO: remove this hack, after arm PID coefficients are tuned
    Command raiseArm = new RaiseArm(m_arm, aimArmAngle, ArmConstants.kExtraDelayForOscillationsToStop, null, setAngleFromSmartDashboardKey);
    Command shoot = new Shoot(m_shooter, m_intake, m_arm, shootingFlywheelRpm);
    Command raiseAfterwardsToSaveEnergy = new RequestArmAngle(m_arm, ArmConstants.kArmAgleToSaveEnergy);

    Command result = new SequentialCommandGroup(dropArm, raiseArm, shoot, raiseAfterwardsToSaveEnergy);

    Command keepWheelsOnXBrake = m_drivetrain.run(m_drivetrain::setX); // keep wheels on X brake, otherwise opponent robots can easily disrupt aiming

    return result.deadlineWith(keepWheelsOnXBrake);
  }

  private Command makeApproachAndShootCommand(double aimArmAngle, double shootingFlywheelRpm, String setAngleFromSmartDashboardKey) {
    // very close to target: ty=+12, tx=-12
    // 1.5 robot lenghts away: ty=0, tx=-7
    // very far away: ty=-5.5, tx=-3
    double approachSpeed = -0.3, seekingSpeed = 0.1; // set them to zero if you want to just aim
    var approachAndAim = new FollowVisualTarget.WhenToFinish(0, 12, 0, true);
    var aim = new FollowVisualTarget(
      m_drivetrain, m_aimingCamera, CameraConstants.kSpeakerPipelineIndex,
      seekingSpeed, approachSpeed,
      CameraConstants.kAimingCameraImageRotation,
      approachAndAim);

    var raiseAndShoot = makeRaiseAndShootCommand(aimArmAngle, shootingFlywheelRpm, setAngleFromSmartDashboardKey);
    var raiseAndShootIfFound = raiseAndShoot.onlyIf(aim::getEndedWithTarget);

    return new SequentialCommandGroup(aim, raiseAndShootIfFound);
  }

  private Command makeBrakeAndShootCommand() {
    // -- aiming horizontally
    double approachSpeed = 0.0; // do not approach, just aim
    double seekingSpeed = 0.0; // do not seek, just aim
    var dontDriveJustAim = new FollowVisualTarget.WhenToFinish(0, 0, 0, true);
    var aim = new FollowVisualTarget(
      m_drivetrain, m_aimingCamera, CameraConstants.kSpeakerPipelineIndex,
      seekingSpeed, approachSpeed,
      CameraConstants.kAimingCameraImageRotation,
      dontDriveJustAim);


    // -- aiming vertically and shooting, with wheels locked in X position
    double initialDropAngle = 22;
    double lowestPossibleFiringAngle = 37;
    double shootingFlywheelRpm = 5700;

    Command dropArm = new RaiseArm(m_arm, initialDropAngle, 0); // TODO: maybe comment out dropArm, and see if determinism breaks?
    Command raiseArm = new RaiseArm(m_arm, lowestPossibleFiringAngle, ArmConstants.kExtraDelayForOscillationsToStop, this::getGoodFiringAngle, null);
    Command shoot = new Shoot(m_shooter, m_intake, m_arm, shootingFlywheelRpm);
    Command raiseAfterwardsToSaveEnergy = new RequestArmAngle(m_arm, ArmConstants.kArmAgleToSaveEnergy);
    Command raiseArmAndShoot = new SequentialCommandGroup(dropArm, raiseArm, shoot, raiseAfterwardsToSaveEnergy);

    Command keepWheelsOnXBrake = m_drivetrain.run(m_drivetrain::setX); // keep wheels on X brake, otherwise opponent robots can easily disrupt aiming
    Command raiseArmAndShootWithWheelsLocked = raiseArmAndShoot.deadlineWith(keepWheelsOnXBrake);


    Command shootIfAimed = raiseArmAndShootWithWheelsLocked.onlyIf(aim::getEndedWithTarget);
    return new SequentialCommandGroup(aim, shootIfAimed);
  }

  // gets a good angle for the shooter arm, given what aiming camera says about angular distance to target
  private double getGoodFiringAngle() {
    double targetY = m_aimingCamera.getLastValidY();
    SmartDashboard.putNumber("brakeAndShootTargetY", targetY);
    if (targetY == 0)
      return ArmConstants.initialMaxAngle; // something safe, if we don't see the target anymore

    // targetY angle tells us about how far the target is
    // use the firing table to find the parfect firing angle for this distance to target
    double firingAngle = FieldMap.kSpeakerFiringTable.interpolate(targetY);
    SmartDashboard.putNumber("brakeAndShootFiringAngle", firingAngle);
    return firingAngle;
  }

  private Command makeEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp, 0); // is 94 a good angle to eject the note into the amp reliably? 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp); // is 80 a good angle for pushing the note in
    Command result = new SequentialCommandGroup(raiseArm, ejectAndPush);
    return result;
  }

  private Command makeConsistentEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp, 0); // is 94 a good angle to eject the note into the amp reliably? 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp); // is 80 a good angle for pushing the note in
    Command raiseEjectAndPush = new SequentialCommandGroup(raiseArm, ejectAndPush);

    // to do it more reliably, ensure that the bumper has fully approached the amp *and* ensure consistent starting angle 
    Command dropToLowerAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp - 5, 0); // TODO: remove, this is a hack until arm PID coeffs are tuned
    Command raiseToPushAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp, 0); // good starting angle
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

  private Command makeApproachNoteCommand(double armAngleAfterPickup) {
    var raiseArm = new RaiseArm(m_arm, 80, 0);

    var whenToStop = new FollowVisualTarget.WhenToFinish(-16, 0, 0, false);

    var approachAndAim = new FollowVisualTarget(
      m_drivetrain, m_pickupCamera, CameraConstants.kNotePipelineIndex, CameraConstants.kNoteApproachRotationSpeed, CameraConstants.kNoteApproachSpeed,
      CameraConstants.kPickupCameraImageRotation, whenToStop);
    
    var thenPickup = makePickupNoteCommand(true, armAngleAfterPickup);

    return new SequentialCommandGroup(raiseArm, approachAndAim, thenPickup);
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
    return new RaiseArm(m_arm, ArmConstants.kArmAgleToSaveEnergy, 0);
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
