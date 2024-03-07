// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIDs.*;

public class PDH extends SubsystemBase {
  PowerDistribution m_pdh = new PowerDistribution(kPDH,ModuleType.kRev);

  /** Creates a new PowerDistribution. */
  public PDH() {
    setSwitchableChannel(false);
  }

  public void setSwitchableChannel(boolean enabled){ 
    m_pdh.setSwitchableChannel(enabled);
    System.out.println("switchable channel:" + enabled);
  }

  public void setSwitchableChannelOn() { 
    setSwitchableChannel(true);
  }

  public void setSwitchableChannelOff() {
    setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
