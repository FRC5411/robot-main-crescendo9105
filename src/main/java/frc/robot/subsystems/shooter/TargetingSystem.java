// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  private Translation3d speakerOpeningBlue = new Translation3d(0.26, 5.49, 2.045);
  private Translation3d speakerOpeningRed = new Translation3d(16.26, 5.49, 2.045);

  /**
   * Tree Map that represents the robot's horizontal (X) distance from the Speaker (meters) and the
   * optimal launch angle (degrees)
   */
  private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

  /** Initialize targeting system */
  public TargetingSystem() {
    initializeLaunchMap();
  }

  /** Initialize the launch map */
  private void initializeLaunchMap() {
    launchMap.put(0.0, 55.0);
    launchMap.put(0.1, 52.0);
    launchMap.put(0.2, 50.0);
    launchMap.put(0.3, 49.0);
    launchMap.put(0.4, 49.0);
    launchMap.put(0.5, 48.0);
    launchMap.put(0.6, 47.0);
    launchMap.put(0.7, 46.0);
    launchMap.put(0.8, 45.0);
    launchMap.put(0.9, 44.0);
    launchMap.put(1.0, 43.7);
    launchMap.put(1.1, 42.9);
    launchMap.put(1.2, 42.0);
    launchMap.put(1.3, 41.0);
    launchMap.put(1.4, 40.0);
    launchMap.put(1.5, 39.5);
    launchMap.put(1.6, 39.0);
    launchMap.put(1.7, 37.5);
    launchMap.put(1.8, 36.5);
    launchMap.put(1.9, 35.0);
    launchMap.put(2.0, 34.4);
  }

  /** Returns the optimal angle given the robot's current pose */
  public Rotation2d getLaunchMapAngle(Pose2d robotPose) {
    double distanceFromTargetM =
        // Distances are flipped for both alliances
        (DriverStation.getAlliance().get() == Alliance.Blue)
            // Subtract minor offset not accounted for when getting shot data
            ? (robotPose.getX() - speakerOpeningBlue.getX()) - 0.62
            : (speakerOpeningRed.getX() - robotPose.getX()) - 0.62;

    Rotation2d angle = Rotation2d.fromDegrees(launchMap.get(distanceFromTargetM));

    return angle;
  }
}
