package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SmartMotionShooter extends SubsystemBase {
  private static final int kSmartMotionSlot = 0;

  private final CANSparkFlex m_leadMotor;
  private final CANSparkFlex m_followMotor;
  private final RelativeEncoder m_encoder;

  // static???
  private static SparkPIDController m_pidController;
  private static double m_velocityGoal = 0;

  public double getVelocityGoal() {
    return m_velocityGoal;
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  /*
   * Set the veloicty goal in motor RPM
   */
  public void setVelocityGoal(double rpm) {
    if (rpm == m_velocityGoal)
      return;
    if (rpm < initialMinVel || rpm > initialMaxVel) {
      System.out.println("Shooter Illegal Velocity Goal: " + rpm);
      return;
    }
    System.out.println("Shooter: " + rpm);
    m_velocityGoal = rpm;
    m_pidController.setReference(m_velocityGoal, CANSparkFlex.ControlType.kVelocity);
  }

  /** Creates a new SmartMotionShooter. */
  public SmartMotionShooter() {
    m_leadMotor = new CANSparkFlex(Constants.CANIDs.kShooterMotorA, MotorType.kBrushless);
    m_leadMotor.restoreFactoryDefaults();
    m_leadMotor.setInverted(true);
    m_leadMotor.setIdleMode(IdleMode.kCoast);
    
    m_followMotor = new CANSparkFlex(Constants.CANIDs.kShooterMotorB, MotorType.kBrushless);
    m_followMotor.restoreFactoryDefaults();
    m_followMotor.follow(m_leadMotor, false);
    m_followMotor.setIdleMode(IdleMode.kCoast);

    // initialze PID controller and encoder objects
    m_pidController = m_leadMotor.getPIDController();
    m_encoder = m_leadMotor.getEncoder();

    // set PID coefficients
    m_pidController.setP(initialP);
    m_pidController.setI(initialI);
    m_pidController.setD(initialD);
    m_pidController.setIZone(initialIz);
    m_pidController.setFF(initialFF);
    m_pidController.setOutputRange(initialMinOutput, initialMaxOutput);

    m_pidController.setSmartMotionMaxVelocity(initialMaxVel, kSmartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(initialMinVel, kSmartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(initialMaxAcc, kSmartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(initialAllowedError, kSmartMotionSlot);
  }

  // will it work in autonomous? and let's test if it is needed at all
  public static void teleopPeriodic() {
    m_pidController.setReference(m_velocityGoal, CANSparkFlex.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterRpmSeen", getVelocity());
    SmartDashboard.putNumber("shooterRpmGoal", getVelocityGoal());
  }

  public void Stop() {
    m_leadMotor.stopMotor();
  }

}
