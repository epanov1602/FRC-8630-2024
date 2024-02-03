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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimToVisualTarget;
import frc.robot.commands.GoToPoint;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

        private static LimelightCamera m_camera = new LimelightCamera();
 
        // The driver's controller
        private XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                Runnable joystickDriveControl = null;
                if (Constants.DriveConstants.kCopterJoystickLayout) {
                        // copter layoyt: right stick for movement, left stick for rotation
                        joystickDriveControl = () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                Constants.DriveConstants.kFieldRelative,
                                                                true);
                } else {
                        // tank layoyt: left stick for movement, right stick for rotation 
                        joystickDriveControl = () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                                                Constants.DriveConstants.kFieldRelative,
                                                                true);
                }

                // Configure default teleop commands
                m_robotDrive.setDefaultCommand(new RunCommand(joystickDriveControl, m_robotDrive));
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
                Command goThereAndFaceWest = GoToPoint.create(m_robotDrive, 3, -1, 90);
                JoystickButton btnB = new JoystickButton(m_driverController, Button.kB.value);
                btnB.onTrue(goThereAndFaceWest); // go to this point and face West

                Command comeBackAndFaceNorth = GoToPoint.create(m_robotDrive, 0.2, 0.0, 0);
                JoystickButton btnA = new JoystickButton(m_driverController, Button.kA.value);
                btnA.onTrue(comeBackAndFaceNorth); // come back in front of zero to zero and face North
        
                Command aimToTag = new AimToVisualTarget(m_robotDrive, m_camera, 0, true, true); 
                JoystickButton btnY = new JoystickButton(m_driverController, Button.kY.value);
                btnY.onTrue(aimToTag); 

                Command parkingBrake = new RunCommand(() -> m_robotDrive.setX(), m_robotDrive);
                JoystickButton btnX = new JoystickButton(m_driverController, Button.kX.value);
                btnX.whileTrue(parkingBrake);

                Command resetOdometry = new ResetOdometry(m_robotDrive)
                JoystickButton btnRightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
                btnRightBumper.onTrue(resetOdometry);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new AimToVisualTarget(m_robotDrive, m_camera, 0, false, false);
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
