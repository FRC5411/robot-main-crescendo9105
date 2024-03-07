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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  private Translation3d speakerOpeningBlue = new Translation3d(0.23, 5.58, 2.045);
  private Translation3d speakerOpeningRed = new Translation3d(16.26, 5.58, 2.045);

  private final double LAUNCH_MAP_OFFSET_M = 0.93 + 0.46 - 0.23;

  /**
   * Tree Map that represents the robot's horizontal (X) distance from the Speaker (meters) and the
   * optimal launch angle (degrees)
   */
  private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

  /** Initialize targeting system */
  public TargetingSystem() {
    initializeLaunchMap();

    // Initialize log tables
    Logger.recordOutput("Shooter/TargetingSystem/Angle", new Rotation2d());
    Logger.recordOutput("Shooter/TargetingSystem/Heading", new Rotation2d());
    Logger.recordOutput("Shooter/TargetingSystem/Distance", 0.0);
  }

  /** Initialize the launch map */
  private void initializeLaunchMap() {
    launchMap.put(0.0 + LAUNCH_MAP_OFFSET_M, 55.0);
    launchMap.put(0.25 + LAUNCH_MAP_OFFSET_M, 52.0);
    launchMap.put(0.50 + LAUNCH_MAP_OFFSET_M, 48.0);
    launchMap.put(0.75 + LAUNCH_MAP_OFFSET_M, 45.0);
    launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 42.5);
    launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 40.0);
    launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 38.5);
    launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 37.0);
    launchMap.put(2.0 + LAUNCH_MAP_OFFSET_M, 35.6);
    launchMap.put(2.25 + LAUNCH_MAP_OFFSET_M, 34.5);
    launchMap.put(2.5 + LAUNCH_MAP_OFFSET_M, 33.5);
    launchMap.put(2.75 + LAUNCH_MAP_OFFSET_M, 32.0);
    launchMap.put(3.0 + LAUNCH_MAP_OFFSET_M, 31.1);
    launchMap.put(3.25 + LAUNCH_MAP_OFFSET_M, 30.3);
    launchMap.put(3.5 + LAUNCH_MAP_OFFSET_M, 29.9);
  }

  /** Returns the optimal angle given the robot's current pose */
  public Rotation2d getLaunchMapAngle(Pose2d robotPose) {
    Rotation2d angle =
        Rotation2d.fromDegrees(launchMap.get(calculateSpeakerDistanceM(robotPose).getAsDouble()));

    Logger.recordOutput("Shooter/TargetingSystem/Angle", angle);

    return angle;
  }

  /** Returns the optimal heading for shooting */
  public Rotation2d getOptimalLaunchHeading(Pose2d robotPose) {
    // Must add 180 since the front of the robot is the intake, not the shooter
    Rotation2d heading =
        (DriverStation.getAlliance().get() == Alliance.Blue)
            ? new Rotation2d(
                speakerOpeningBlue.getX() - robotPose.getX(),
                speakerOpeningBlue.getY() - robotPose.getY())
            : new Rotation2d(
                    speakerOpeningRed.getX() - robotPose.getX(),
                    speakerOpeningRed.getY() - robotPose.getY())
                .plus(Rotation2d.fromDegrees(180.0));

    Logger.recordOutput("Shooter/TargetingSystem/Heading", heading);

    return heading;
  }

  /** Calculate the tangental distance from the speaker */
  private DoubleSupplier calculateSpeakerDistanceM(Pose2d currentRobotPose) {
    double distanceM =
        (DriverStation.getAlliance().get() == Alliance.Blue)
            ? Math.hypot(
                speakerOpeningBlue.getX() - currentRobotPose.getX(),
                speakerOpeningBlue.getY() - currentRobotPose.getY())
            : Math.hypot(
                speakerOpeningRed.getX() - currentRobotPose.getX(),
                speakerOpeningRed.getY() - currentRobotPose.getY());

    Logger.recordOutput("Shooter/TargetingSystem/Distance", distanceM);

    return () -> distanceM;
  }
}
