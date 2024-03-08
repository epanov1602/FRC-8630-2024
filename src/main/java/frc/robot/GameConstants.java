package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;

public final class GameConstants {
  // initial position of robot on the field
  public static final double kInitialX = 1.53 * 0.33;
  public static final double kInitialY = 1.44 * 0.33;
  public static final double kInitialShootingHeadingDegrees = -30;
  public static final double kInitialHeadingDegrees = 0; // should be around 0 for blue and around 180 for red

  // autonomous escape trajectory to take, after all the pieces are scored
  public static final List<Translation2d> kAutonomousEscapeTrajectory = FieldMap.rescale(0.33, FieldMap.kBlueApproachCenerlineFromRight); // can be = null, if no escape is needed
  public static final double kAutonomousEscapeFinalHeading = 0; // where to point at the end of autonomous escape
}

