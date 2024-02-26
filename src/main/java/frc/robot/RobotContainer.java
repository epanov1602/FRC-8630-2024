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
  //private CommandXboxController m_manipulatorController = new CommandXboxController(OIConstants.kManipulatorController);

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
    /*
    m_aimingCamera.setDefaultCommand(m_aimingCamera.run(() -> {
      if (m_intake.isNoteInside())
         m_aimingCamera.setLightOn();
      else
         m_aimingCamera.setLightOff();
    }));
    */
  }

  private void tankJoystickDrive() {
    // tank layoyt: left stick for movement, right stick for rotation
    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_driverJoystick.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getRightX(), OIConstants.kDriveDeadband),
        Constants.DriveConstants.kFieldRelative,
        true);

    // cameras should be in driver camera mode, if operator is active on sticks
    if (operatorUsingSticks()) {
      m_pickupCamera.setDriverCameraMode();
      m_aimingCamera.setDriverCameraMode();
    }
  }

  private void copterJoystickDrive() {
    // if the left stick is down, wiggle drive
    if (m_driverJoystick.getLeftY() > 0.3) {
      m_drivetrain.wiggleDrive(m_driverJoystick.getRightY(), 0.2, 0.5); // 0.2 rotation speed, 0.5 seconds per wiggle
      return;
    }

    // if keeping high pressure on the throttle stick, not field-relative anymore (for manual aiming)
    double slowDownFactor = 1.0;
    boolean fieldRelative = Constants.DriveConstants.kFieldRelative;
    if (m_driverJoystick.getLeftY() < -0.3) {
      fieldRelative = false;
      slowDownFactor = 0.3;
    }

    // copter layoyt otherwise: right stick for movement, left stick for rotation
    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_driverJoystick.getRightY() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getRightX() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverJoystick.getLeftX() * slowDownFactor, OIConstants.kDriveDeadband),
        fieldRelative,
        true);

    // cameras should be in driver camera mode, if operator is active on sticks
    if (operatorUsingSticks()) {
      m_pickupCamera.setDriverCameraMode();
      m_aimingCamera.setDriverCameraMode();
    }
  }


  private FollowVisualTarget makeApproachNoteCommand() {
    double approachSpeed = 1.0;
    boolean doneIfNoMoreMovesToMake = true;
    double stopIfTargetLowerThanDegrees = -9.5;
    double seekSpeed = 0.1; // seek right and left if not looking
    var whenToStop = new FollowVisualTarget.WhenToFinish(stopIfTargetLowerThanDegrees, 0, 0, doneIfNoMoreMovesToMake);
    return new FollowVisualTarget(m_drivetrain, m_pickupCamera, CameraConstants.kNotePipelineIndex, seekSpeed, approachSpeed, Rotation2d.fromDegrees(0), whenToStop);
  }

  private Command makeApproachSpeakerCommand() {
    double approachSpeed = -0.3;
    boolean doneIfNoMoreMovesToMake = true;
    double stopIfTargetHigherThanDegrees = 11;
    double seekSpeed = 0.1; // seek right and left if not looking
    var whenToStop = new FollowVisualTarget.WhenToFinish(0, stopIfTargetHigherThanDegrees, 0, doneIfNoMoreMovesToMake);
    return new FollowVisualTarget(m_drivetrain, m_aimingCamera, CameraConstants.kSpeakerPipelineIndex, seekSpeed, approachSpeed, Rotation2d.fromDegrees(0), whenToStop);
  }

  private Command makeWiggleiggleDriveCommand() {
    SmartDashboard.setDefaultNumber("wiggleRotSpeed", 0.4);
    SmartDashboard.setDefaultNumber("wiggleTranSpeed", 0.09);
    SmartDashboard.setDefaultNumber("wiggleInterval", 0.5);
    var resetWiggles = m_drivetrain.runOnce(() ->m_drivetrain.resetWiggleDrive());
    var wiggle = m_drivetrain.run(() -> {
      double rot = SmartDashboard.getNumber("wiggleRotSpeed", 0.0);
      double trans = SmartDashboard.getNumber("wiggleTranSpeed", 0.0);
      double interval = SmartDashboard.getNumber("wiggleInterval", 0.0);
      m_drivetrain.wiggleDrive(trans, rot, interval);
    });
    return resetWiggles.andThen(wiggle);
  }

  private void configureButtonBindings() {
    m_driverJoystick.a().whileTrue(makeWiggleiggleDriveCommand());
    m_driverJoystick.b().whileTrue(makeApproachNoteCommand());
    m_driverJoystick.x().whileTrue(m_drivetrain.run(m_drivetrain::setX));
    m_driverJoystick.y().whileTrue(makeApproachSpeakerCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindingsComplete() {
    // button Y to yank note from the shooter
    m_driverJoystick.y().onTrue(new EjectFromShooter(m_shooter, m_intake, m_arm)); // new ResetOdometry(m_drivetrain)); /* for the actual game, replace with 

    // button X to lock wheels in "X" position and just shoot
    m_driverJoystick.x().whileTrue(m_drivetrain.run(m_drivetrain::setX));
    // make it more complex? wheels in X and shoot at 2000 rpm: new ParallelCommandGroup(m_drivetrain.run(m_drivetrain::setX), new Shoot(m_shooter, m_intake, 2000));

    // POV down: pick up the piece using just arm (but not automatically driving towards it)
    Command pickUpWithoutDriving = makePickupNoteCommand(false, 30); // raise arm by 30 degrees after pickup
    m_driverJoystick.povDown().whileTrue(pickUpWithoutDriving);

    // POV left: pick up the piece using arm and drivetrain (automatically driving towards gamepiece)
    Command pickUpWithDriving = makePickupNoteCommand(true, 30); // raise arm by 30 degrees after pickup
    m_driverJoystick.povLeft().whileTrue(pickUpWithDriving);

    // POV right: drop the arm tp 15 degrees and eject the note with speed 0.5
    Command dropArm = new RaiseArm(m_arm, 15);
    Command eject = new EjectNote(m_intake, 0.5);
    m_driverJoystick.povRight().whileTrue(new SequentialCommandGroup(dropArm, eject));

    // POV up: raise, aim and shoot at angle 120, rpm 2000 (scoring target is nearby)
    Command raiseAndShoot = makeAimAndShootCommand(120, CameraConstants.kSpeakerPipelineIndex, 2000);
    m_driverJoystick.povUp().whileTrue(raiseAndShoot);
  
    // left bumper: go to feeder station and try to pickup a note using vision (human player can throw that note right when robot approaches)
    Command goToFeeder = new SwerveTrajectoryToPoint(m_drivetrain, FieldMap.kBlueApproachFeederWhileHuggingWall, Rotation2d.fromDegrees(0));
    Command prepareToLookForNotes = new SwitchVisualTarget(m_pickupCamera, CameraConstants.kNotePipelineIndex);
    Command pickUpNote = makePickupNoteCommand(true, 30 /* 30 degree arm angle after successful pickup */);
    Command pickUpNoteFromFeeder = new SequentialCommandGroup(prepareToLookForNotes, goToFeeder, pickUpNote);
    Command pickUpNoteFromFeederSafely = pickUpNoteFromFeeder.until(this::operatorUsingSticks);
    m_driverJoystick.leftBumper().whileTrue(pickUpNoteFromFeederSafely);

    // right bumper: return to the spearker and score that note
    Command returnToSpeaker = new SwerveTrajectoryToPoint(m_drivetrain, FieldMap.kBlueRetreatFromCenerlineAlongRightWall, Rotation2d.fromDegrees(-45)); // face -45 degrees at the end, for aiming
    Command prepareToLookForSpeaker = new SwitchVisualTarget(m_aimingCamera, CameraConstants.kSpeakerPipelineIndex);
    Command aimAndShoot = makeAimAndShootCommand(110, CameraConstants.kSpeakerPipelineIndex, 2000 /* shoot at 2000 rpm */);
    Command returnAndScore = new SequentialCommandGroup(prepareToLookForSpeaker, returnToSpeaker, aimAndShoot);
    Command returnAndScoreSafely = returnAndScore.until(this::operatorUsingSticks);
    m_driverJoystick.rightBumper().whileTrue(returnAndScoreSafely);

    // button B: before opponents do the same, race to the center line and grab one note and waise it all the way up (90 degrees)
    Command trajectoryToCenterLine = new SwerveTrajectoryToPoint(m_drivetrain, FieldMap.kBlueApproachCenerlineFromRight, Rotation2d.fromDegrees(45));
    Command goFindOneNote = makeFindAndHaulAwayCommand(10 /* arm angle after pickup */, trajectoryToCenterLine, new RaiseArm(m_arm, 90));
    m_driverJoystick.b().whileTrue(goFindOneNote);

    // finally, button A: do what the autonomous mode would do (but interruptable, because only running while pushed)
    Command copyOfAutonomous = getAutonomousCommand();
    m_driverJoystick.a().whileTrue(copyOfAutonomous);
  }


  private Command makeAimAndShootCommand(double aimArmAngle, int aimVisualTargetIndex, double shootingFlywheelRpm) {
    // 1. aim while raising the arm
    // -- aim visually to the AprilTag below target
    FollowVisualTarget.WhenToFinish finishAimingIfNotMoving = new FollowVisualTarget.WhenToFinish(0, 0, 0, true);
    Command aim = new FollowVisualTarget(m_drivetrain, m_aimingCamera, aimVisualTargetIndex, 0, aimVisualTargetIndex, CameraConstants.kAimingCameraImageRotation, finishAimingIfNotMoving);
    // -- raise
    RaiseArm raiseArm = new RaiseArm(m_arm, aimArmAngle);

    // in parallel: aim + raise
    ParallelRaceGroup raiseAndAim = new ParallelRaceGroup(aim, raiseArm); // maybe also start slowly accelerating the shooter in parallel here?

    // 2. shoot only if camera has been seeing the target (otherwise this will be a waste)
    Command shoot = new Shoot(m_shooter, m_intake, shootingFlywheelRpm);
    Command shootIfAimedSuccessfully = shoot.onlyIf(m_aimingCamera::isTargetRecentlySeen);

    // connect 1 + 2
    Command aimAndShoot = new SequentialCommandGroup(raiseAndAim, shootIfAimedSuccessfully);
    return aimAndShoot;
  }

  private Command makeRetreatAimAndShootCommand(Command retreatTrajectory, double aimArmAngle, int aimVisualTargetIndex, double shootingFlywheelRpm) {
    // 1. we already have a retreat trajectory

    // 2. make the aim+shoot command
    Command aimAndShoot = makeAimAndShootCommand(aimArmAngle, aimVisualTargetIndex, shootingFlywheelRpm);

    // connect 1 + 2
    Command retreatAimAndShoot = new SequentialCommandGroup(retreatTrajectory, aimAndShoot);
    return retreatAimAndShoot;
  }

  /**
   * Makes a command to pick up the note
   */
  private Command makePickupNoteCommand(boolean driveTowards, double armAngleAfterPickup) {
    // 1. take the note
    Command grabNote;
    if (driveTowards == true)
      grabNote = new IntakeNote(m_intake, m_arm, m_drivetrain, armAngleAfterPickup);
    else /* if driveTowards==false, we are not supposed to drive towards the note, so do not let the command use m_drivetrain */
      grabNote = new IntakeNote(m_intake, m_arm, null, 0);

    // 2. after the note is in, it might be blocking the shooter from spinning: move it back by a few inches
    Command unblockShooter = new EjectNote(m_intake, 0.3).withTimeout(0.2); // speed=30%, and add timeout=0.2

    // 1 + 2
    Command result = new SequentialCommandGroup(grabNote, unblockShooter);
    return result;
  }

  /**
   * Makes a command to approach the note using a pickup camera, and then pick it up
   */
  private Command makeApproachAndPickupNoteCommand(double armAngleAfterPickup) {
    // 1. approach using video
    var approach = makeApproachNoteCommand();

    // 2. pick up
    Command pickup = makePickupNoteCommand(true, armAngleAfterPickup);
    // wait! only pick up if the visual approach actually ended with the target (note) in sight
    Command pickupIfFound = pickup.onlyIf(approach::getEndedWithTarget);

    // 1 + 2
    Command approachAndPickup = new SequentialCommandGroup(approach, pickupIfFound);
    return approachAndPickup;
  }

  /**
   * Makes a command to approach the note using a pickup camera, and then pick it up, and haul away
   * @param haulAwayCommand if not null, set it 
   */
  private Command makeApproachPickupAndHaulAwayCommand(double armAngleAfterPickup, Command haulAwayCommand) {
    // 1. approach and pickup
    Command approachAndPickup = makeApproachAndPickupNoteCommand(armAngleAfterPickup);

    // 2. haul away *if* note was picked up
    Command haulAwayIfPickedUp = haulAwayCommand.onlyIf(() -> m_intake.isNoteInside());

    // 1 + 2
    Command pickupAndHaulAway = new SequentialCommandGroup(approachAndPickup, haulAwayIfPickedUp);
    return pickupAndHaulAway;
  }

  /**
   * Makes a command to go looking for a note along approach trajectory command, pick up using camera, and haul away using some haul away command
   */
  private Command makeFindAndHaulAwayCommand(double armAngleAfterPickup, Command approachTrajectory, Command haulAwayCommand) {
    // 0. make camera look for note before starting the trajectory
    var makeCameraLookForNote = new SwitchVisualTarget(m_pickupCamera, CameraConstants.kNotePipelineIndex);

    // 1. look for note until camera notices it
    var followTrajectoryUntilNoteFoundOnCamera = approachTrajectory.until(m_pickupCamera::isTargetRecentlySeen);

    // 2. pickup and haul away
    var pickupAndHaulAway = makeApproachPickupAndHaulAwayCommand(20, haulAwayCommand); // is 20 degrees a good angle for haul away?
  
    // but! only pick-up-and-haul-away should only happen if target was visually noticed
    var pickupAndHaulAwayIfFound  = pickupAndHaulAway.onlyIf(m_pickupCamera::isTargetRecentlySeen);

    // 1 + 2
    Command goFetch = new SequentialCommandGroup(makeCameraLookForNote, followTrajectoryUntilNoteFoundOnCamera, pickupAndHaulAwayIfFound);
    return goFetch;
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
    // how we go there
    Command trajectoryAlongCenterLine = new SwerveTrajectoryToPoint(m_drivetrain, FieldMap.kBlueApproachCenerlineFromRight, Rotation2d.fromDegrees(45));

    // how we come back
    Command trajectoryReturnToSpeaker = new SwerveTrajectoryToPoint(m_drivetrain, FieldMap.kBlueRetreatFromCenerlineAlongRightWall, Rotation2d.fromDegrees(-45)); // face -45 degrees at the end, for aiming
    Command retreatAndShoot = makeRetreatAimAndShootCommand(trajectoryReturnToSpeaker, 10, CameraConstants.kSpeakerPipelineIndex, 2000 /* shoot at 2000 rpm */);

    // connect them
    Command getOneNoteFromCenterAndScoreIt = makeFindAndHaulAwayCommand(10 /* arm angle after pickup */, trajectoryAlongCenterLine, retreatAndShoot);
    return getOneNoteFromCenterAndScoreIt;
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
