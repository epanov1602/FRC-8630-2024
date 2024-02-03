package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ProtocolException;
import java.net.UnknownHostException;
import java.io.IOException;
 
/**
 * Limelight camera sensor for FRC Romi
 */ 

public class LimelightCamera {
  protected NetworkTable m_table;
  private NetworkTableEntry m_tx, m_ty, m_ta, m_pipeline;

  public double getX() { return m_tx.getDouble(0.0); }
  public double getY() { return m_ty.getDouble(0.0); }
  public double getA() { return m_ta.getDouble(0.0); }

  public int getPipeline() { return (int)m_pipeline.getDouble(-1); }
  public void setPipeline(int pipeline) { m_pipeline.setDouble(pipeline); }

  /** Create a new RomiLimelight. */
  public LimelightCamera() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_pipeline = m_table.getEntry("pipeline");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
  }
}
