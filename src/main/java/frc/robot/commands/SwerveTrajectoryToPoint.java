// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryToPoint extends Command {
  private final DriveSubsystem m_drivetrain;
  private final List<Translation2d> m_waypoints;
  private final Pose2d m_targetPose;

  private Command m_command = null;
  private boolean m_gotToTheEnd = false;

  public SwerveTrajectoryToPoint(DriveSubsystem drivetrain, List<Translation2d> waypoints, Pose2d targetPose) {
    m_drivetrain = drivetrain;
    m_waypoints = waypoints;
    m_targetPose = targetPose;
    addRequirements(drivetrain);
  }

  public SwerveTrajectoryToPoint(DriveSubsystem drivetrain, List<Translation2d> waypoints, Rotation2d targetHeading) {
    // last waypoint will become our target, it targetHeading is given instead of targetPose
    Translation2d lastWaypoint = waypoints.get(waypoints.size() - 1);
  
    ArrayList<Translation2d> allWaypointsExceptLast = new ArrayList<Translation2d>();
    for (Translation2d waypoint : waypoints) {
      if (waypoint != lastWaypoint)
        allWaypointsExceptLast.add(waypoint);
    }

    m_waypoints = allWaypointsExceptLast;
    m_targetPose = new Pose2d(lastWaypoint, targetHeading);

    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public boolean getGotToTheEnd() { return m_gotToTheEnd; }

  private Command createCommand() {
    // A. if we are pretty close to the destination, just use a simple SwerveToPoint command
  
    Pose2d start = m_drivetrain.getPose();
    double distanceToTarget = start.getTranslation().getDistance(m_targetPose.getTranslation());
    if (distanceToTarget < 2 * AutoConstants.kDistanceTolerance)
      return new SwerveToPoint(m_drivetrain, m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getDegrees(), false);

    // B. otherwise, use the proper WPILib trajectory command:

    // make a subset of waypoints: skip all the waypoints that are already behind us, only include those that are on the way to the target
    ArrayList<Translation2d> subsetOfWaypoints = new ArrayList<Translation2d>();

    // -- which waypoint is the nearest?
    Translation2d nearestWaypoint = null;
    double nearestWaypointDistance = m_targetPose.getTranslation().getDistance(start.getTranslation());
    // ^^ start with distance to the end of trajectory (if no waypoint is nearer than this, we can just go straight to the end, no waypoints needed)
    for (Translation2d waypoint : m_waypoints) {
      double distance = waypoint.getDistance(start.getTranslation());
      if (distance < nearestWaypointDistance) {
        nearestWaypointDistance = distance;
        nearestWaypoint = waypoint;
      }
    }

    // -- skip all the waypoints up to the nearest, but include all subsequent ones (all the way to the target)
    boolean include = false;
    for (Translation2d waypoint : m_waypoints) {
      if (include == true)
        subsetOfWaypoints.add(waypoint);
      if (waypoint == nearestWaypoint)
        include = true;
    }

    // -- finally, create trajectory command from the remaining waypoints and the final target
    // (copied from https://youtu.be/0Xi9yb1IMyA, thanks Team 6814)

    // 1. generate trajectory
    TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    trajectoryConfig = trajectoryConfig.setKinematics(DriveConstants.kDriveKinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, subsetOfWaypoints, m_targetPose, trajectoryConfig);

    // 2. define PID controllers for catching up with this trajectory
    PIDController xController = new PIDController(AutoConstants.kPTranslationForTrajectory, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPTranslationForTrajectory, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPRotationForTrajectory, 0, 0, AutoConstants.kRotationControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 3. construct the command to follow this trajectory using these PID controllers
    return new SwerveControllerCommand(
        trajectory,
        m_drivetrain::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // interrupt the previous comand, if it was still there
    if (m_command != null)
      end(true); 

    // generate the next command that we will need now
    m_command = createCommand();
    m_command.initialize();
    m_gotToTheEnd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_command != null)
      m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_command != null)
      m_command.end(interrupted);
    m_command = null;
    m_drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_command == null)
      return true; // no command, probably finished some time ago
    if (m_command.isFinished()) {
      m_gotToTheEnd = true;
      return true;
    }
    // otherwise, we have not finished
    return false;
  }
}
