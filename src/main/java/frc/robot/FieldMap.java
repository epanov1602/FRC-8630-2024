// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.CubicSpline;

/** Add your docs here. */
public final class FieldMap {

  // -- blue starting points
  public static final Pose2d kBlueStartRight = new Pose2d(1.60, 4.40, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueStartMiddle = new Pose2d(1.55, 5.55, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueStartLeft = new Pose2d(1.60, 6.60, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueSpeaker = new Pose2d(0.2, 5.55, Rotation2d.fromDegrees(0));

  // -- red starting points
  public static final Pose2d kRedStartLeft = new Pose2d(16.50 - 1.60, 4.40, Rotation2d.fromDegrees(180));
  public static final Pose2d kRedStartMiddle = new Pose2d(16.50 - 1.55, 5.55, Rotation2d.fromDegrees(180));
  public static final Pose2d kRedStartRight = new Pose2d(16.50 - 1.60, 6.60, Rotation2d.fromDegrees(180));
  public static final Pose2d kRedSpeaker = new Pose2d(16.50 - 0.2, 5.55, Rotation2d.fromDegrees(180));

  // -- blue centerline approaches
  public static final List<Translation2d> kBlueApproachCenerlineFromRight = rescale(1.0, List.of(
    new Translation2d(1.60, 4.40),
    new Translation2d(2.52, 2.53),
    new Translation2d(3.81, 1.56),
    new Translation2d(5.21, 1.03),
    new Translation2d(6.43, 0.85),
    new Translation2d(7.17, 0.85)
  ));

  public static final List<Translation2d> kBlueApproachCenerlineFromLeft = rescale(1.0, List.of(
    new Translation2d(1.60, 6.59),
    new Translation2d(3.83, 7.00),
    new Translation2d(5.38, 7.37),
    new Translation2d(7.46, 7.47)
  ));

  public static final List<Translation2d> kBlueApproachCenerlineFromMiddle = rescale(1.0, List.of(
    new Translation2d(1.55, 5.54),
    new Translation2d(5.82, 6.70),
    new Translation2d(7.30, 5.82)
  ));



  // -- red centerline approaches
  public static final List<Translation2d> kRedApproachCenerlineFromLeft = rescale(1.0, List.of(
    new Translation2d(16.50 - 1.60, 4.40),
    new Translation2d(16.50 - 2.52, 2.53),
    new Translation2d(16.50 - 3.81, 1.56),
    new Translation2d(16.50 - 5.21, 1.03),
    new Translation2d(16.50 - 6.43, 0.85),
    new Translation2d(16.50 - 7.17, 0.85)
  ));

  public static final List<Translation2d> kRedApproachCenerlineFromRight = rescale(1.0, List.of(
    new Translation2d(16.50 - 1.60, 6.59),
    new Translation2d(16.50 - 3.83, 7.00),
    new Translation2d(16.50 - 5.38, 7.37),
    new Translation2d(16.50 - 7.46, 7.47)
  ));

  public static final List<Translation2d> kRedApproachCenerlineFromMiddle = rescale(1.0, List.of(
    new Translation2d(16.50 - 1.55, 5.54),
    new Translation2d(16.50 - 5.82, 6.70),
    new Translation2d(16.50 - 7.30, 5.82)
  ));


  // helpful function
  public static List<Translation2d> reverse(List<Translation2d> t) {
    List<Translation2d> trajectory = new ArrayList<Translation2d>(t);
    Collections.reverse(trajectory);
    return trajectory;
  }

  public static List<Translation2d> rescale(double factor, List<Translation2d> t) {
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
   *   6.16: 40.5
   *   9.37: 37
   *   12.5: 32
   */

   // 714
   public static final CubicSpline kSpeakerFiringTable714 = new CubicSpline(
     new double[] { -1.52, 0.29,  2.77, 6.16, 9.37, 12.5 },
     new double[] {    52.5, 50.0, 46, 40.5,   37,   32 }
   );

   // shabazz
   public static final CubicSpline kSpeakerFiringTableShabazz = new CubicSpline(
     new double[] { 0,  2.77, 6.16, 9.37, 12.5, 17.91 },
     new double[] { 53.5, 49,  44.5, 40.5,  39, 34.5  }
   );
}
