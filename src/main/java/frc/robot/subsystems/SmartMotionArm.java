/**
 * +090 degrees is straight up
 *  000 degrees is horizontal forward
 * +180 degrees is horizontal backwards
 * 
 * > +180 never legal
 * <  000 never legal
 */
package frc.robot.subsystems;

import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import static frc.robot.Constants.ArmConstants.initialAllowedError;
import static frc.robot.Constants.ArmConstants.initialD;
import static frc.robot.Constants.ArmConstants.initialFF;
import static frc.robot.Constants.ArmConstants.initialI;
import static frc.robot.Constants.ArmConstants.initialIz;
import static frc.robot.Constants.ArmConstants.initialMaxAcc;
import static frc.robot.Constants.ArmConstants.initialMaxAngle;
import static frc.robot.Constants.ArmConstants.initialMaxOutput;
import static frc.robot.Constants.ArmConstants.initialMaxVel;
import static frc.robot.Constants.ArmConstants.initialMinAngle;
import static frc.robot.Constants.ArmConstants.initialMinOutput;
import static frc.robot.Constants.ArmConstants.initialMinVel;
import static frc.robot.Constants.ArmConstants.initialP;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SmartMotionArm extends SubsystemBase {
  private static final boolean kEncoderInverted = false;
  private static final double kEncoderPositionFactor = 360; // degrees
  private static final double kEncoderVelocityFactor = 360 / 60; // degrees/second
  // ^^ using degrees here

  private final CANSparkMax m_leadMotor; // right side
  private final CANSparkMax m_followMotor; // left side
  private final SparkPIDController m_pidController;
  private final SparkAbsoluteEncoder m_encoder; // through-bore connected to follow SparkMax
  private final SparkLimitSwitch m_forwardLimit;
  private final SparkLimitSwitch m_reverseLimit;
  public double kP, kI, kD, kIz, kFF, maxOutput, minOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, maxAngle,
      minAngle;

  private double m_angleGoal = initialMinAngle;


  public double getAngleGoal() {
    return m_angleGoal;
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public double getAngleVelocity() {
    return m_encoder.getVelocity();
  }

  /*
   * Set the position goal in angle, >= 0
   */
  public void setAngleGoal(double angle) {
    m_angleGoal = angle;
    if (m_angleGoal < minAngle)
      m_angleGoal = minAngle;
    if (m_angleGoal > maxAngle)
      m_angleGoal = maxAngle;
    System.out.println("setAngleGoal: angle=" + angle + ", angleGoal=" + m_angleGoal);
    m_pidController.setReference(getAngleGoal(), CANSparkMax.ControlType.kSmartMotion);
    m_pidController.setIMaxAccum(0.02, 0); // do not allow the integral PID term accumulate too much
    m_pidController.setIAccum(0);
  }

  public SmartMotionArm() {
    m_leadMotor = new CANSparkMax(Constants.CANIDs.kArmMotorRight, MotorType.kBrushless);
    m_leadMotor.restoreFactoryDefaults();
    m_leadMotor.setInverted(true);
    m_leadMotor.setIdleMode(IdleMode.kBrake);

    m_forwardLimit = m_leadMotor.getForwardLimitSwitch(kNormallyOpen);
    m_reverseLimit = m_leadMotor.getReverseLimitSwitch(kNormallyOpen);

    m_followMotor = new CANSparkMax(Constants.CANIDs.kArmMotorLeft, MotorType.kBrushless);
    m_followMotor.restoreFactoryDefaults();
    m_followMotor.follow(m_leadMotor, true);
    m_followMotor.setIdleMode(IdleMode.kBrake);

    // initialze PID controller and encoder objects
    m_pidController = m_leadMotor.getPIDController();
    m_encoder = m_leadMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_encoder.setPositionConversionFactor(kEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(kEncoderVelocityFactor);
    m_encoder.setInverted(kEncoderInverted);

    // PID coefficients
    kP = initialP;
    kI = initialI;
    kD = initialD;
    kIz = initialIz;
    kFF = initialFF;
    maxOutput = initialMaxOutput;
    minOutput = initialMinOutput;

    // Smart Motion Coefficients
    maxVel = initialMaxVel; // rpm
    minVel = initialMinVel; // rpm
    maxAcc = initialMaxAcc;
    allowedErr = initialAllowedError;
    maxAngle = initialMaxAngle;
    minAngle = initialMinAngle;

    // set PID coefficients
    m_pidController.setP(kP); // 0 - 3
    m_pidController.setI(kI); // 0 - 3
    m_pidController.setD(kD); // 0 - 3
    m_pidController.setIZone(kIz); // 0 - infinity
    m_pidController.setFF(kFF); // 0 - 3
    m_pidController.setOutputRange(minOutput, maxOutput); // -1 - 1

    int smartMotionSlot = 0;
    m_pidController.setFeedbackDevice(m_encoder);
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();

    // and our first angle goal
    setAngleGoal(minAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angleSeen", m_encoder.getPosition());
    SmartDashboard.putNumber("angleWanted", getAngleGoal());
    SmartDashboard.putNumber("angleVeloccity", m_encoder.getVelocity());
  }

  public void stop() {
    m_leadMotor.stopMotor();
  }

  /**
   * Get the current status of the Forward and Reverse limit switches
   * 
   * @return boolean[]{Fwd, Rev}
   */
  public boolean[] getLimitSwitches() {
    return new boolean[] { m_forwardLimit.isPressed(), m_reverseLimit.isPressed() };
  }

}
