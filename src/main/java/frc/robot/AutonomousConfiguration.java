package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutonomousConfiguration {
    public final String m_name;
    public final Pose2d m_initialPose;
    public final double m_initialHeadingToSpeaker;
    public final double m_headingToSecondPickupNote;
    public final List<Translation2d> m_escapeTrajectory;
  
    public AutonomousConfiguration(String name, Pose2d initialPose, double headingToSpeaker, double headingToSecondPickupNote, List<Translation2d> escapePath) {
      m_name = name;
      m_initialPose = initialPose;
      m_initialHeadingToSpeaker = headingToSpeaker;
      m_headingToSecondPickupNote = headingToSecondPickupNote;
      m_escapeTrajectory = escapePath;
    }

    // lots of pre-baked configs
    public static final AutonomousConfiguration kBlueLeft =
      new AutonomousConfiguration("Blue1", FieldMap.kBlueStartLeft, 210, -45, FieldMap.kBlueApproachCenerlineFromLeft);

    public static final AutonomousConfiguration kBlueMiddle =
      new AutonomousConfiguration("Blue2", FieldMap.kBlueStartMiddle, 180, 45, FieldMap.kBlueApproachCenerlineFromMiddle);

    public static final AutonomousConfiguration kBlueRight =
      new AutonomousConfiguration("Blue3", FieldMap.kBlueStartRight, 150, 45, FieldMap.kBlueApproachCenerlineFromMiddle);

    public static final AutonomousConfiguration kRedLeft =
      new AutonomousConfiguration("Red1", FieldMap.kRedStartLeft, 30, 135, FieldMap.kRedApproachCenerlineFromLeft);

    public static final AutonomousConfiguration kRedMiddle =
      new AutonomousConfiguration("Red2", FieldMap.kRedStartMiddle, 0, 135, FieldMap.kRedApproachCenerlineFromMiddle);

    public static final AutonomousConfiguration kRedRight =
      new AutonomousConfiguration("Red3", FieldMap.kRedStartRight, -30, -135, FieldMap.kRedApproachCenerlineFromRight);
  }
