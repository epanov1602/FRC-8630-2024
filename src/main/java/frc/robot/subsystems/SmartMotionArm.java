/**
 * 000 degrees is horizontal forward
 * 090 degrees is straight up
 * 180 degrees is horizontal backwards
 * 270 degrees is straight down
 * 
 * 000 - 290 never legal
 * 290 - 360: subtract 360
 */
package frc.robot.subsystems;

import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;
import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.TravelMode;
import pabeles.concurrency.IntOperatorTask.Min;

public class SmartMotionArm extends SubsystemBase {
  private CANSparkMax motor;
  private SparkPIDController pidController;
  private RelativeEncoder m_encoder;
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  public double kP, kI, kD, kIz, kFF, maxOutput, minOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, maxAngle,
      minAngle;
  private double angleGoal;
  private double startingAngle = 74.0;//-17; // real is about 74

  public double getAngleGoal() {
    return angleGoal;
  }

  /*
   * Set the position goal in angle, >= 0
   */
  public void setAngleGoal(double angle) {
    //("setAngleGoal:" + angle);
    if (!IsAngleGoalValid(angle)) return;
    angleGoal = angle;
  }

  public boolean IsAngleGoalValid(double angle) {
    if (angle < minAngle || angle > maxAngle) {
      System.out.println("Illegal Arm Angle Goal: " + angle);
      return false;
    }
    return true;
  }

  public SmartMotionArm() {
    angleGoal = startingAngle;
    motor = new CANSparkMax(Constants.CANIDs.kArmMotor, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);

    m_forwardLimit = motor.getForwardLimitSwitch(kNormallyOpen);
    m_reverseLimit = motor.getReverseLimitSwitch(kNormallyOpen);
    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);

    // initialze PID controller and encoder objects
    pidController = motor.getPIDController();
    m_encoder = motor.getEncoder();
    resetEncoders();

    // PID coefficients
    kP = initialP;
    kI = initialI;
    kD = initialD;
    kIz = initialIz;
    kFF = initialFF;
    maxOutput = initialMaxOutput;
    minOutput = initialMinOutput;
    maxRPM = initialMaxRPM;

    // Smart Motion Coefficients
    maxVel = initialMaxVel; // rpm
    minVel = initialMinVel; // rpm
    maxAcc = initialMaxAcc;
    allowedErr = initialAllowedError;
    maxAngle = initialMaxAngle;
    minAngle = initialMinAngle;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(minOutput, maxOutput);

    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  @Override
  public void periodic() { 
    double setPoint = getAngleGoal() * motorRevolutionsPerDegree;
    pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  /*
   * Must be called before using arm, preferably from robotInit.
   */
  public void initialize() {
    FindHome();
  }

  /*
   * Moves onto the Home switch and just off of it
   */
  public void FindHome() {
  }

  public void Stop() {
    motor.stopMotor();
  }

  public void enable() {
  }

  public void disable() {
  }

  /**
   * Get the current status of the Forward and Reverse limit switches
   * 
   * @return boolean[]{Fwd, Rev}
   */
  public boolean[] getLimitSwitches() {
    // return new boolean[] { m_forwardLimit.isPressed(), m_reverseLimit.isPressed()
    // };
    return new boolean[] { false, false };
  }

  public void resetEncoders() {
    m_encoder.setPosition(startingAngle * motorRevolutionsPerDegree);
  }

  /**
   * @return current angle in radians
   */
  public double getCurrentRadians() {
    return getCurrentDegrees() * Math.PI / 180;
  }

  /**
   * @return current angle in radians
   */
  public double getCurrentDegrees() {
    return (m_encoder.getPosition() / motorRevolutionsPerDegree);
  }
}
