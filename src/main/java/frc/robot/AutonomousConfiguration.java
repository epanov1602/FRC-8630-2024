package frc.robot;

import java.util.List;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutonomousConfiguration {
    public final Pose2d m_initialPose;
    public final double m_initialHeadingToSpeaker;
    public final double m_headingToSecondPickupNote;
    public final List<Translation2d> m_escapeTrajectory;
  
    public AutonomousConfiguration(Pose2d initialPose, double headingToSpeaker, double headingToSecondPickupNote, List<Translation2d> escapePath) {
      m_initialPose = initialPose;
      m_initialHeadingToSpeaker = headingToSpeaker;
      m_headingToSecondPickupNote = headingToSecondPickupNote;
      m_escapeTrajectory = escapePath;
    }

    public static AutonomousConfiguration get(AllianceStationID stationId) {
      if (stationId == AllianceStationID.Blue1)
        return new AutonomousConfiguration(FieldMap.kBlueStartLeft, 210, -45, FieldMap.kBlueApproachCenerlineFromLeft);
      if (stationId == AllianceStationID.Blue2)
        return new AutonomousConfiguration(FieldMap.kBlueStartMiddle, 180, 45, FieldMap.kBlueApproachCenerlineFromMiddle);
      if (stationId == AllianceStationID.Blue3)
        return new AutonomousConfiguration(FieldMap.kBlueStartRight, 150, 45, FieldMap.kBlueApproachCenerlineFromMiddle);

      if (stationId == AllianceStationID.Red1)
        return new AutonomousConfiguration(FieldMap.kRedStartLeft, 30, 135, FieldMap.kRedApproachCenerlineFromLeft);
      if (stationId == AllianceStationID.Red2)
        return new AutonomousConfiguration(FieldMap.kRedStartMiddle, 0, 135, FieldMap.kRedApproachCenerlineFromMiddle);
      if (stationId == AllianceStationID.Red3)
        return new AutonomousConfiguration(FieldMap.kRedStartRight, -30, -135, FieldMap.kRedApproachCenerlineFromRight);
      
      // otherwise, we don't know
      return null;
    }
  }
