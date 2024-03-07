package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;

public final class GameConstants {
  // initial position of robot on the field
  public static final double kInitialX = 1.90;
  public static final double kInitialY = 4.50;
  public static final double kInitialHeadingDegrees = 0; // should be around 0 for blue and around 180 for red

  // autonomous escape trajectory to take, after all the pieces are scored
  public static final List<Translation2d> kAutonomousEscapeTrajectory = FieldMap.kBlueApproachCenerlineFromLeft; // can be = null, if no escape is needed
  public static final double kAutonomousEscapeFinalHeading = 0; // where to point at the end of autonomous escape
}

