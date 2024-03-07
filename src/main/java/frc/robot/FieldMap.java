// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.CubicSpline;

/** Add your docs here. */
public final class FieldMap {

  // -- blue starting points
  public static final Translation2d kBlueApproachSpeakerFromRight = new Translation2d(1.90, 4.35);
  public static final Translation2d kBlueApproachSpeakerFromCenter = new Translation2d(1.90, 5.55);
  public static final Translation2d kBlueApproachSpeakerFromLeft = new Translation2d(1.90, 6.75);
  public static final Translation2d kBlueSpeaker = new Translation2d(0.2, 5.55);

  // -- red starting points
  public static final Translation2d kRedApproachSpeakerFromLeft = new Translation2d(14.65, 4.35);
  public static final Translation2d kRedApproachSpeakerFromCenter = new Translation2d(14.65, 5.55);
  public static final Translation2d kReadApproachSpeakerFromRight = new Translation2d(14.65, 6.75);
  public static final Translation2d kRedSpeaker = new Translation2d(16.3, 5.55);

  // -- blue centerline approaches
  public static final List<Translation2d> kBlueApproachCenerlineFromRight = rescale(1.0, List.of(
    new Translation2d(1.90, 4.10),
    new Translation2d(2.57, 2.73),
    new Translation2d(3.71, 1.47),
    new Translation2d(5.16, 0.71),
    new Translation2d(6.52, 0.65),
    new Translation2d(7.25, 1.26)
  ));

  public static final List<Translation2d> kBlueRetreatFromCenerlineAlongRightWall = reverse(kBlueApproachCenerlineFromRight);

  public static final List<Translation2d> kBlueApproachCenerlineFromLeft = rescale(1.0, List.of(
    new Translation2d(1.90, 6.10),
    new Translation2d(3.83, 7.00),
    new Translation2d(5.39, 7.39),
    new Translation2d(6.32, 7.51),
    new Translation2d(7.27, 5.37)
  ));

  public static final List<Translation2d> kBlueRetreatFromCenerlineAlongLeftWall = reverse(kBlueApproachCenerlineFromLeft);


  // -- red centerline approaches
  public static final List<Translation2d> kRedApproachCenerlineFromLeft = rescale(1.0, List.of(
    new Translation2d(16.50 - 1.90, 4.10),
    new Translation2d(16.50 - 2.57, 2.73),
    new Translation2d(16.50 - 3.71, 1.47),
    new Translation2d(16.50 - 5.16, 0.71),
    new Translation2d(16.50 - 6.52, 0.65),
    new Translation2d(16.50 - 7.25, 1.26)
  ));

  public static final List<Translation2d> kRedRetreatFromCenerlineAlongLeftWall = reverse(kBlueApproachCenerlineFromRight);

  public static final List<Translation2d> kRedApproachCenerlineFromRight = rescale(1.0, List.of(
    new Translation2d(16.50 - 1.90, 6.10),
    new Translation2d(16.50 - 3.83, 7.00),
    new Translation2d(16.50 - 5.39, 7.39),
    new Translation2d(16.50 - 6.32, 7.51),
    new Translation2d(16.50 - 7.27, 5.37)
  ));

  public static final List<Translation2d> kRedRetreatFromCenerlineAlongRightWall = reverse(kRedApproachCenerlineFromLeft);



  // helpful function
  private static List<Translation2d> reverse(List<Translation2d> t) {
    List<Translation2d> trajectory = new ArrayList<Translation2d>(t);
    Collections.reverse(trajectory);
    return trajectory;
  }

  private static List<Translation2d> rescale(double factor, List<Translation2d> t) {
    List<Translation2d> trajectory = new ArrayList<Translation2d>(t.size());
    for (Translation2d waypoint : t)
      trajectory.add(waypoint.times(factor));
    return trajectory;
  }


  /* firing table (angle to tag -> angle to raise the arm):
   * aimAngleToFiringAngle:
   *   -1.5: 52.5
   *   0.29: 50.0
   *   2.77: 46
   *   6.16: 41.5
   *   9.37: 37
   *   12.5: 32
   */

   public static final CubicSpline kSpeakerFiringTable = new CubicSpline(
     new double[] { -1.52, 0.29,  2.77, 6.16, 9.37, 12.5 },
     new double[] {    52.5, 50.0, 46, 41.5,   37,   32 }
   );
}
