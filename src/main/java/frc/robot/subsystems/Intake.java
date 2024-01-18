// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;

import frc.robot.Constants;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkLimitSwitch;

public class Intake extends SubsystemBase {
  private static final int CANID = Constants.CANIDs.kIntakeMotor;
  private SparkLimitSwitch m_forwardLimit;

  private CANSparkFlex m_motor;

  /** Creates a new Intake. */
  public Intake() {

    m_motor = new CANSparkFlex(CANID, kBrushless);
    m_motor.restoreFactoryDefaults();
    // todo N.O. for testing & development - N.C. for production KSM 2024-02-18
    m_forwardLimit = m_motor.getForwardLimitSwitch(kNormallyOpen);
    m_forwardLimit.enableLimitSwitch(true);
  }

  /**
   * Enable or disable limit switch
   * @param state
   */
  public void limitSwitchEnable(boolean state){
    m_forwardLimit.enableLimitSwitch(state);  
  }

  /**
   * Disable limit switch and feed note to shooter.
   */
  public void feedToShooter(){
    m_forwardLimit.enableLimitSwitch(false);
    m_motor.set(0.5);
  }

  /**
   * Enable the limit switch and turn on motor
   */
  public void intakeNote(){
    m_forwardLimit.enableLimitSwitch(true);
    setSpeed(0.5);
  }

  /**
   * Set the motor speed and display speed on rio log
   * @param speed
   */
  public void setSpeed(double speed) {
    m_motor.set(speed);
    System.out.println("Intake: " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
