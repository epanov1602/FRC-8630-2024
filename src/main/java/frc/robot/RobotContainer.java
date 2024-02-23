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

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.Constants.OdometryConstants;
import frc.robot.commands.AimToDirection;
import frc.robot.commands.FollowVisualTarget;
import frc.robot.commands.SwerveToPoint;
import frc.robot.commands.SwerveTrajectoryToPoint;
import frc.robot.commands.SwitchVisualTarget;
import frc.robot.commands.GoToPoint;
import frc.robot.commands.MockPickupCommand;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static LimelightCamera m_pickupCamera = new LimelightCamera(CameraConstants.kPickupCameraName);

  // The driver's controller
  private XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default teleop commands
    if (Constants.DriveConstants.kCopterJoystickLayout)
      m_robotDrive.setDefaultCommand(new RunCommand(this::copterJoystickDrive, m_robotDrive));
    else
      m_robotDrive.setDefaultCommand(new RunCommand(this::tankJoystickDrive, m_robotDrive));
  }

  private void tankJoystickDrive() {
    // tank layoyt: left stick for movement, right stick for rotation
    m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        Constants.DriveConstants.kFieldRelative,
        true);
  }

  private void copterJoystickDrive() {
    double slowDownFactor = 1.0;
    boolean fieldRelative = Constants.DriveConstants.kFieldRelative;
    if (m_driverController.getLeftY() > 0.3) {
      m_robotDrive.wiggleDrive(m_driverController.getRightY(), MathUtil.clamp(m_driverController.getLeftX(), -0.2, 0.2), 0.5 /* 0.5 seconds per wiggle */);
      return;
    }

    if (m_driverController.getLeftY() < -0.3) {
      fieldRelative = false; // meant to keep high pressure on the throttle stick? not field-relative anymore but moving fast
      slowDownFactor = 0.3;
    }

    // copter layoyt: right stick for movement, left stick for rotation
    m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getRightY() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX() * slowDownFactor, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX() * slowDownFactor, OIConstants.kDriveDeadband),
        fieldRelative,
        true);
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
  private void configureButtonBindings() {
    Command resetOdometry = new ResetOdometry(m_robotDrive);
    JoystickButton btnY = new JoystickButton(m_driverController, Button.kY.value);
    btnY.onTrue(resetOdometry);

    Command goNorth = new SequentialCommandGroup(
        new SwerveToPoint(m_robotDrive, 1, 0, 0, true),
        new SwerveToPoint(m_robotDrive, 2, 1, 90, false));
    Command goNorthSafely = goNorth.until(this::operatorUsingSticks);
    JoystickButton btnX = new JoystickButton(m_driverController, Button.kX.value);
    btnX.onTrue(goNorthSafely);

    Command goBack = new SwerveToPoint(m_robotDrive, 0, 0, 0, false);
    JoystickButton btnB = new JoystickButton(m_driverController, Button.kB.value);
    btnB.onTrue(goBack);

    Command aimToTag = new FollowVisualTarget(
      m_robotDrive, m_pickupCamera, 9, 0.1, 0.6,
      CameraConstants.kPickupCameraImageRotation,
      new FollowVisualTarget.WhenToFinish(-14, 0, 0, true));
    JoystickButton btnA = new JoystickButton(m_driverController, Button.kA.value);
    btnA.onTrue(aimToTag);

    // a slightly bigger command: return while hugging the left side of the field

    Command retreatUsingLeftSideOfTheField = new SwerveTrajectoryToPoint(
      m_robotDrive,
      List.of(
        new Translation2d(0.25, 0.0),
        new Translation2d(0.5, 0.0),
        new Translation2d(0.75, 0.0),
        new Translation2d(1.0, 0.0),
        new Translation2d(1.25, 0.0),
        new Translation2d(1.375, 1.0),
        new Translation2d(1.5, 1.5),
        new Translation2d(1.75, 1.5)
      ),
      new Pose2d(2, 1.5, Rotation2d.fromDegrees(-90)));
    JoystickButton btnLeftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);    
    btnLeftBumper.onTrue(retreatUsingLeftSideOfTheField);

    // and even bigger: a command run around until gamepiece (note) is acquired on camera, and then approach that note and pick up

    // -- part 0: start detecting gamepiece target (note)
    var switchToDetectingNote = new SwitchVisualTarget(m_pickupCamera, CameraConstants.kNotePipelineIndex);

    // -- part 1: running around until target detected reliably
    var runAroundLooking = new SwerveTrajectoryToPoint(
      m_robotDrive,
      List.of(
        new Translation2d(0.0, 1),
        new Translation2d(0.5, 0),
        new Translation2d(1.0, 1),
        new Translation2d(1.5, 0),
        new Translation2d(2, 1)
      ),
      new Pose2d(1, -0.8, Rotation2d.fromDegrees(-170)));

    Command runAroundUntilDetected = runAroundLooking.until(
      () -> m_pickupCamera.getPercentageOfTimeTargetDetected() > 0.5
    );

    // -- part 2: approach the target at speed no more than 0.4 of full throttle (or rotate at rate 0.1 while seeking)
    var approachTarget = new FollowVisualTarget(
      m_robotDrive, m_pickupCamera, CameraConstants.kNotePipelineIndex, 0.1, 0.4, CameraConstants.kPickupCameraImageRotation,
      new FollowVisualTarget.WhenToFinish(-14, 0, 0, true));
    var pickupIfApproached = new MockPickupCommand(m_robotDrive).onlyIf(
      () -> approachTarget.getFinishedWithTarget() == true
    );
    var approachAndPickup = new SequentialCommandGroup(approachTarget, pickupIfApproached);

    var startApproachAndPickupIfDetected = approachAndPickup.onlyIf(
      () -> runAroundLooking.getGotToTheEnd() == false /* only approach the target if the runaround above resulted in finding */
    );

    // -- finally, the whole sequence together
    var findAndApproachTarget = new SequentialCommandGroup(
      switchToDetectingNote,
      runAroundUntilDetected,
      startApproachAndPickupIfDetected
    );

    // (but make it interruptible by operator touching sticks to take control)
    var findAndApproachSafely = findAndApproachTarget.until(this::operatorUsingSticks);

    JoystickButton btnRightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);    
    btnRightBumper.onTrue(findAndApproachSafely);
  }

  private boolean operatorUsingSticks() {
    double totalInput = Math.abs(m_driverController.getLeftX()) + Math.abs(m_driverController.getLeftY())
        + Math.abs(m_driverController.getRightX()) + Math.abs(m_driverController.getRightY());
    return totalInput > 0.1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var whenToFinishAiming = new FollowVisualTarget.WhenToFinish(-16, 0, 0, false);
    return new FollowVisualTarget(m_robotDrive, m_pickupCamera, 9, 0.1, 0.1, Rotation2d.fromDegrees(-30), whenToFinishAiming);
  }

  public static void testInit() {
    // run testInit() on each subsystem.
    m_robotDrive.testInit();
  }

  public static void testPeriodic() {
    // run testPeriodic() on each subsystem
    m_robotDrive.testPeriodic();
  }
}
