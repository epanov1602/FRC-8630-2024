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
    m_motor.setInverted(true);//forward = intake
    // todo N.O. for testing & development - N.C. for production KSM 2024-02-18
    m_forwardLimit = m_motor.getForwardLimitSwitch(kNormallyOpen);
    m_forwardLimit.enableLimitSwitch(true);
  }

  /**
   * Returns true if the note is inside (for deciding when the note was successfully picked up, or successfully released)
   */
  public boolean isNoteInside() { 
    return m_forwardLimit.isPressed();
  }

  /**
   * Enable or disable limit switch
   * @param state
   */
  public void limitSwitchEnable(boolean state){
    m_forwardLimit.enableLimitSwitch(state);  
  }

  /**
   * Enable the limit switch and turn on motor forward
   */
  public void intakeNote(){
    m_forwardLimit.enableLimitSwitch(true);
    setSpeed(0.3);
  }

  /**
   * Enable the limit switch and turn on motor forward
   */
  public void ejectNote(double speed){
    if (speed < 0.1)
      speed = 0.1;
    if (speed > 1.0)
      speed = 1.0;
    m_forwardLimit.enableLimitSwitch(false);
    setSpeed(-speed);
  }

  /**
   * Disable the limit switch and turn on motor forward
   */
  public void feedNoteToShooter(){
    m_forwardLimit.enableLimitSwitch(false);
    setSpeed(0.3);
  }

  /**
   * Stop, but do not change limit switch state
   */
  public void stop(){
    setSpeed(0);
  }
 
  /**
   * Set the motor speed and display speed on rio log
   * @param speed
   */
  private void setSpeed(double speed) {
    m_motor.set(speed);
    System.out.println("Intake: " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
