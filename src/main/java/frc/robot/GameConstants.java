package frc.robot;

import java.util.List;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class GameConstants {
  public static final int numAutonomousNotesToScore = 1;

  // initial position of robot on the field
  public static final double kInitialX = 1.53;
  public static final double kInitialY = 1.44;
  public static final double kInitialHeadingDegrees = 0; // should be around 0 for blue and around 180 for red
  public static final double kInitialShootingHeadingDegrees = -30;

  // autonomous escape trajectory to take, after all the pieces are scored
  public static final List<Translation2d> kAutonomousEscapeTrajectory = FieldMap.rescale(1.0, FieldMap.kBlueApproachCenerlineFromRight); // can be = null, if no escape is needed
  public static final double kAutonomousEscapeFinalHeading = 0; // where to point at the end of autonomous escape
}
