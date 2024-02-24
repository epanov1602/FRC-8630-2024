// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; 
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CANIDs {
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final int kIntakeMotor = 9;
    public static final int kShooterMotorA = 10;
    public static final int kShooterMotorB = 11;
    public static final int kArmMotorLeft = 12;
    public static final int kArmMotorRight = 13;
  }

  public final class OdometryConstants {
    // initial position of robot on the field (does not have to be {x=0, y=0})
    public static final double kInitialX = 0;
    public static final double kInitialY = 0;
    public static final double kInitialHeadingDegrees = 0;
  }

  public static final class DriveConstants {
    public static final boolean kFieldRelative = true;
    public static final boolean kCopterJoystickLayout = true;

    /****  Chassis configuration ****/
    public static final double kTrackWidth = Units.inchesToMeters(24.0);
    public static final double kWheelBase = Units.inchesToMeters(27.25);
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // radians per second
    public static final double kPhysicalMaxAccelerationMetersPerSecondSquared = 5; // to avoid slipping when on floor
    
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 0.36 * kPhysicalMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeed = 0.5 * kPhysicalMaxAngularSpeedRadiansPerSecond; // radians per second
    public static final double kMaxRateLimitedAngularSpeed = 0.25 * kPhysicalMaxAngularSpeedRadiansPerSecond; // smaller turn speed when under human control
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.6 * kPhysicalMaxAccelerationMetersPerSecondSquared;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared / Math.sqrt(kWheelBase * kWheelBase + kTrackWidth * kTrackWidth);

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.7;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kDrivingMinSpeedMetersPerSecond = 0.01; // avoids oscillations at low battery going in-and-out of brownout (might need to be lower)
  }

  public static final class OIConstants {

    /**
     * Use XBox controller or MS Flight Control Joystick for testing and
     * development.
     */
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorController = 1; // second joystick for the manipulator control

    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    // simple driving and turning

    // turning
    public static final double kPRotation = 0.06 / DriveConstants.kMaxAngularSpeed; // proportional gain for rotation: if set too high, robot will overshoot
    public static final double kMaxTurningSpeed = 0.99;
    public static final double kMinTurningSpeed = 0.03; // any value lower than this causes motors to not spin at all
    public static final double kDirectionToleranceDegrees = 3; // plus minus 3 degrees of direction tolerance is ok
    public static final double kTurningSpeedToleranceDegreesPerSecond = 7; // if the chassis is moving slower than this and facing the right way, we can stop aiming
      
    // driving
    public static final double kPTranslation = 3.2 / DriveConstants.kMaxSpeedMetersPerSecond; // proportional gain for forward motion: if set too high, robot will overshoot
    public static final double kMaxForwardSpeed = 0.99;
    public static final double kMinForwardSpeed = 0.07; // any value lower than this causes motors to not spin at all
    public static final double kDistanceTolerance = 0.1; // meters
    public static final double kForwardSpeedTolerance = 0.05; // meters per second
    public static final double kOversteerFactor = 0.33; // between 0 and 1 please
    public static final double kOversteerCapDegrees = 45; // do not oversteer by more than this much

    // swerve trajectory following
    public static final double kPTranslationForTrajectory = 1.5; // translation gain for catching up with trajectory
    public static final double kPRotationForTrajectory = 3; // rotation gain for catching up with trajectory
    public static final TrapezoidProfile.Constraints kRotationControllerConstraints =
      new TrapezoidProfile.Constraints(
            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10,
            DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared / 4);
  }

  public static final class CameraConstants {
    public static final Rotation2d kPickupCameraImageRotation = Rotation2d.fromDegrees(-30);
    public static final String kPickupCameraName = "limelight-pickup";
    public static final int kNotePipelineIndex = 9;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ExtensionConstants {
    // PID coefficients
    public static final double initialP = 5e-5;
    public static final double initialI = 1e-6;
    public static final double initialD = 0;
    public static final double initialIz = 0;
    public static final double initialFF = 0.000156;
    public static final double initialMaxOutput = 1;
    public static final double initialMinOutput = -1;
    public static final double initialMaxRPM = 5700;

    // Smart Motion Coefficients
    public static final double initialMaxVel = 2000; // rpm
    public static final double initialMinVel = -2000; // rpm
    public static final double initialMaxAcc = 2500;
    public static final double initialAllowedError = .02;
    public static final double initialMaxInches = 15;

    public static final double motorRevolutionsPerInch = 2.105; // empirical

    public static enum TravelMode {
      Velocity,
      Position;
    }

    public static enum LimitSwich {
      Forward,
      Reverse
    }
  }

  /*
     * Initial Shooter values used at startup
   */
  public static final class ShooterConstants {
    // PID coefficients
    public static final double initialP = 5e-5;
    public static final double initialI = 1e-6;
    public static final double initialD = 0;
    public static final double initialIz = 0;
    public static final double initialFF = 0.000156;
    public static final double initialMaxOutput = 1;
    public static final double initialMinOutput = -1;
    public static final double initialMaxRPM = 5700;

    // Smart Motion Coefficients
    public static final double initialMaxVel = 2000; // rpm
    public static final double initialMinVel = -2000; // rpm
    public static final double initialMaxAcc = 2500;
    public static final double initialAllowedError = .02;
      public static final double initialMaxInches = 30;

    private static final double chainPitch = 0.25; // inches
    private static final int chainSprocket = 22; // teeth
    private static final double sprocketCircumfrence = chainPitch * chainSprocket;
    private static final double gearReduction = 15.0;
      private static final double fudgeFactor = 0.80; //adjust using actual measurements
      public static final double motorRevolutionsPerInch = (gearReduction / sprocketCircumfrence)*fudgeFactor;

    public static enum TravelMode {
      Velocity,
      Position;
    }

    public static enum LimitSwich {
      Forward,
      Reverse
    }
  }

  /*
   * Initial Arm values used at startup
   */
  public static final class ArmConstants {
    // PID coefficients
    public static final double initialP = 10e-5;
    public static final double initialI = 0; // 1e-6;
    public static final double initialD = 0;
    public static final double initialIz = 0;
    public static final double initialFF = 0; // 0.000156;
    public static final double initialMaxOutput = 1;
    public static final double initialMinOutput = -1;
    public static final double initialMaxRPM = 5700;

    // Smart Motion Coefficients
    public static final double initialMaxVel = 2000; // rpm
    public static final double initialMinVel = -2000; // rpm
    public static final double initialMaxAcc = 2500;
    public static final double initialAllowedError = .02;
    public static final double initialMaxAngle = +110;
    public static final double initialMinAngle = 0;

    //chainPitch = 0.375; // 3/8 inches; #35 chain
    private static final double chainSprocket = 60; // teeth
    private static final double driveSprocket = 14; // teeth
    private static final double gearReduction = 12.0;
    private static final double chainReduction = chainSprocket / driveSprocket;
    private static final double fudgeFactor = 1; // empirical
    public static final double motorRevolutionsPerDegree = gearReduction * chainReduction / 360 * fudgeFactor;   

    public static enum LimitSwich {
      Forward,
      Reverse
    }
  }

}
