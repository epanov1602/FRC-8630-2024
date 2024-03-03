package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
/**
 * Limelight camera sensor for FRC Romi
 */ 

public class LimelightCamera extends SubsystemBase {
  protected NetworkTable m_table;
  private NetworkTableEntry m_tx, m_ty, m_ta, m_pipeline, m_ledMode, m_camMode, m_percentTimeSeen;
  private boolean m_driverCameraMode = true;
  private double m_exposureWindowSeconds = 0.4;
  private double m_percentageOfTimeSeen = 0;
  private double m_lastTimeLooked = 0;

  public double getA() { return m_ta.getDouble(0.0); }
  public double getX() { return m_tx.getDouble(0.0); }
  public double getY() { return m_ty.getDouble(0.0); }

  public double getX(Rotation2d turn) {
    var point = new Translation2d(getX(), getY()).rotateBy(turn);
    return point.getX();
  }

  public double getY(Rotation2d turn) {
    var point = new Translation2d(getX(), getY()).rotateBy(turn);
    return point.getY();
  }

  public double getPercentageOfTimeTargetDetected() { return m_percentageOfTimeSeen; }
  public boolean isTargetRecentlySeen() { return m_percentageOfTimeSeen > 0.25; }

  public int getPipeline() { return (int)m_pipeline.getDouble(-1); }
  public void setPipeline(int pipeline) { m_pipeline.setDouble(pipeline); m_percentageOfTimeSeen = 0; }
  public void setExposureWindowSeconds(double seconds) { m_exposureWindowSeconds = Math.max(0.05, seconds); }

  public void setLightOn() { m_ledMode.setNumber(0); }
  public void setLightOff() { m_ledMode.setNumber(1); }
  public void setLightFlash() { m_ledMode.setNumber(2); }

  public void setComputerVisionMode() { if (m_driverCameraMode) { m_camMode.setNumber(0); m_driverCameraMode = false; } }
  public void setDriverCameraMode() { if (!m_driverCameraMode) { m_camMode.setNumber(0); m_driverCameraMode = true; } }

  public LimelightCamera(String name) {
    super(fixName(name));
    name = fixName(name);
    m_table = NetworkTableInstance.getDefault().getTable(name);
    m_pipeline = m_table.getEntry("pipeline");
    m_ledMode = m_table.getEntry("ledMode");
    m_camMode = m_table.getEntry("camMode");
    m_percentTimeSeen = m_table.getEntry("percentSeen");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
  }

  private static String fixName(String name) {
    if (name == "" || name == null)
      name = "limelight";
    return name;
  }

  @Override
  public void periodic() {
    // 1. how long has it been since we last looked?
    double now = WPIUtilJNI.now() * 1e-6;
    if (m_lastTimeLooked == 0) {
      m_lastTimeLooked = now;
      return;
    }
    double timeSinceLastLooked = Math.max(0, now - m_lastTimeLooked);
    m_lastTimeLooked = now;

    // 2. update the percentage of time the target is seen
    double targetX = getX();
    if (targetX != 0) {
      // we see the target: the percentage of time goes up, if it was lower than 100%
      m_percentageOfTimeSeen = m_percentageOfTimeSeen + timeSinceLastLooked / m_exposureWindowSeconds;
      if (m_percentageOfTimeSeen > 1)
        m_percentageOfTimeSeen = 1;
    } else {
      // we don't see the target: the percentage of time goes down, if it was higher than 0
      m_percentageOfTimeSeen = m_percentageOfTimeSeen - timeSinceLastLooked / m_exposureWindowSeconds;
      if (m_percentageOfTimeSeen < 0)
        m_percentageOfTimeSeen = 0;
    }
    m_percentTimeSeen.setNumber(m_percentageOfTimeSeen);
  }
}
