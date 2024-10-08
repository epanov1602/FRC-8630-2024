// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OdometryConstants;
import frc.utils.SwerveUtils;




public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      Constants.CANIDs.kFrontLeftDrivingCanId,
      Constants.CANIDs.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      Constants.CANIDs.kFrontRightDrivingCanId,
      Constants.CANIDs.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      Constants.CANIDs.kRearLeftDrivingCanId,
      Constants.CANIDs.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      Constants.CANIDs.kRearRightDrivingCanId,
      Constants.CANIDs.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry m_odometry;

  // Wiggle drive time
  private double m_startWiggleTime = 0;

  /**
   * X speed sent to the SparkMax AFTER limiting
   */
  private double xSpeedCommanded;

  /**
   * X speed sent to the SparkMax AFTER limiting
   */
  private double ySpeedCommanded;

  /**
   * X speed sent to the SparkMax BEFORE limiting
   */
  private double xSpeedRequested;

  /**
   * X speed sent to the SparkMax BEFORE limiting
   */
  private double ySpeedRequested;

  /**
   * Requested rotation
   */
  private double requestedRotation;

  /**
   * true if rates are limited
   */
  private boolean rateLimit;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    try {
      Thread.sleep(1000); // wait for one second while gyro recalibrates
      m_gyro.enableLogging(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    } catch (Exception ex) {
      DriverStation.reportError("Unexpected error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    zeroHeading();

    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
    resetOdometry();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    Pose2d pose = m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("odoX", pose.getX());
    SmartDashboard.putNumber("odoY", pose.getY());
    SmartDashboard.putNumber("odoHeading", pose.getRotation().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the pose from OdometryConstants
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    zeroHeading();
    resetOdometry(new Pose2d(OdometryConstants.kInitialX, OdometryConstants.kInitialY, Rotation2d.fromDegrees(OdometryConstants.kInitialHeadingDegrees)));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
        new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition() },
        pose);
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  public void wiggleDrive(double fwdSpeed, double wiggleRotationSpeed, double wiggleIntervalSeconds) {
    if (m_startWiggleTime == 0)
      m_startWiggleTime = Timer.getFPGATimestamp();

    // how many wiggles have we done already?
    double wiggleCount = (Timer.getFPGATimestamp() - m_startWiggleTime) / wiggleIntervalSeconds;
    if (wiggleCount < 1) {
      // first we try without rotation
      arcadeDrive(fwdSpeed, 0);
    } else if (Math.round(wiggleCount) % 2 == 0) {
      // then if we made number of wiggles, time to wiggle right
      arcadeDrive(fwdSpeed, -wiggleRotationSpeed);
    } else {
      // otherwise if we made odd number of wiggles, time to wiggle left
      arcadeDrive(fwdSpeed, wiggleRotationSpeed);
    }
  }

  public void resetWiggleDrive() {
    m_startWiggleTime = 0;
    arcadeDrive(0, 0);
  }

  public void arcadeDrive(double fwdSpeed, double rotationSpeed, boolean rateLimitForManual) {
    drive(fwdSpeed, 0, rotationSpeed, false, rateLimitForManual);
  }

  public void arcadeDrive(double fwdSpeed, double rotationSpeed) {
    drive(fwdSpeed, 0, rotationSpeed, false, false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimitForManual     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean rateLimitForManual) {

    // if manual, then square the inputs
    if (rateLimitForManual) {
      xSpeed = xSpeed * xSpeed * Math.signum(xSpeed);
      ySpeed = ySpeed * ySpeed * Math.signum(ySpeed);
      rotSpeed = rotSpeed * rotSpeed * Math.signum(rotSpeed);
    }

    this.xSpeedRequested = xSpeed;
    this.ySpeedRequested = ySpeed;
    this.requestedRotation = rotSpeed;

    if (rateLimitForManual) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rotSpeed);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rotSpeed;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;

    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    if (rateLimitForManual)
      rotDelivered = m_currentRotation * DriveConstants.kMaxRateLimitedAngularSpeed; // can be lower, for manual driving

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle() * -1))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * -1).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void testInit() {
    // add items to Shuffleboard
    System.out.println("Drive testInit()");
    SmartDashboard.putData(this);
    // Shuffleboard.
  }

  public void testPeriodic() {
    // System.out.println("Drive testPeriodic()");
    // System.out.println(m_gyro.getAngle()*-1);
    SmartDashboard.putNumber("Angle", m_gyro.getAngle() * -1);
    SmartDashboard.putNumber("xSpeedCommanded", xSpeedCommanded);
    SmartDashboard.putNumber("ySpeedCommanded", ySpeedCommanded);
    SmartDashboard.putNumber("xSpeedRequested", xSpeedRequested);
    SmartDashboard.putNumber("ySpeedRequested", ySpeedRequested);
    SmartDashboard.putNumber("requestedRotation", requestedRotation);
    SmartDashboard.putBoolean("rateLimit", rateLimit);
  }
}
