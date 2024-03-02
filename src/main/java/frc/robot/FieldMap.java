// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public final class FieldMap {


  // -- blue
  public static final Translation2d kBlueApproachSpeakerFromRight = new Translation2d(1.90, 4.40);
  public static final Translation2d kBlueApproachSpeakerFromCenter = new Translation2d(1.90, 4.10);
  public static final Translation2d kBlueApproachSpeakerFromLeft = new Translation2d(1.90, 6.70);

  public static final List<Translation2d> kBlueApproachCenerlineFromRight = rescale(0.33, List.of(
    new Translation2d(1.90, 4.10),
    new Translation2d(2.57, 2.73),
    new Translation2d(3.71, 1.47),
    new Translation2d(5.16, 0.71),
    new Translation2d(6.52, 0.65),
    new Translation2d(7.25, 1.26),
    new Translation2d(7.25, 5.07),
    new Translation2d(7.25, 7.51)
  ));

  public static final List<Translation2d> kBlueRetreatFromCenerlineAlongRightWall = reverse(kBlueApproachCenerlineFromRight);

  public static final List<Translation2d> kBlueApproachCenerlineFromLeft = rescale(0.33, List.of(
    new Translation2d(1.90, 6.10),
    new Translation2d(3.83, 7.00),
    new Translation2d(5.39, 7.39),
    new Translation2d(6.32, 7.51),
    new Translation2d(7.27, 5.37),
    new Translation2d(7.12, 2.97),
    new Translation2d(7.10, 0.82)
  ));

  public static final List<Translation2d> kBlueRetreatFromCenerlineAlongLeftWall = reverse(kBlueApproachCenerlineFromLeft);

  public static final List<Translation2d> kBlueApproachFeederWhileHuggingWall = rescale(0.33, List.of(
    new Translation2d(8.96, 0.80),
    new Translation2d(10.07, 0.80),
    new Translation2d(10.88, 0.80),
    new Translation2d(10.35, 0.92),
    new Translation2d(15.18, 1.57)
  ));


  // -- red



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


  private static List<Translation2d> m_aimingTargetAngleToArmAngle = List.of(
    new Translation2d(12.73, 37),
    new Translation2d(1.37, 50),
    new Translation2d(2.53, 56)
  );

}
