// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  /**
   * Tree Map that represents the robot's horizontal (X) distance from the Speaker (meters) and the
   * optimal launch angle (degrees)
   */
  private static InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

  /** Initialize the launch map */
  public static void initializeLaunchMap() {
    launchMap.put(0.0, 0.0);
  }

  /** Returns the optimal angle given the robot's current pose */
  public static Rotation2d getLaunchMapAngle(Pose2d robotPose) {
    Rotation2d angle = Rotation2d.fromDegrees(launchMap.get(robotPose.getX()));

    return angle;
  }
}
