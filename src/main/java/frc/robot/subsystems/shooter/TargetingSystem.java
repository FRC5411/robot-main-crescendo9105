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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  private static Translation3d speakerOpeningBlue = new Translation3d(0.0, 5.53, 2.045);
  private static Translation3d speakerOpeningRed = new Translation3d(16.26, 5.53, 2.045);

  private static final double LAUNCH_MAP_OFFSET_M = 0.93 + 0.46 - 0.23 - 0.17;
  private static final double LUANCH_MAP_OFFSET_DEGREES = 3.0;

  private static Supplier<Pose2d> currentRobotPose = () -> new Pose2d();

  /**
   * Tree Map that represents the robot's horizontal (X) distance from the Speaker (meters) and the
   * optimal launch angle (degrees)
   */
  private static InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

  /** Initialize the launch map */
  private static void initializeLaunchMap() {
    launchMap.put(0.0 + LAUNCH_MAP_OFFSET_M, 55.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(0.25 + LAUNCH_MAP_OFFSET_M, 52.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(0.50 + LAUNCH_MAP_OFFSET_M, 48.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(0.75 + LAUNCH_MAP_OFFSET_M, 45.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 42.5 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 40.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 38.5 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 37.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(2.0 + LAUNCH_MAP_OFFSET_M, 35.6 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(2.25 + LAUNCH_MAP_OFFSET_M, 34.5 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(2.5 + LAUNCH_MAP_OFFSET_M, 33.5 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(2.75 + LAUNCH_MAP_OFFSET_M, 32.0 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(3.0 + LAUNCH_MAP_OFFSET_M, 31.1 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(3.25 + LAUNCH_MAP_OFFSET_M, 30.3 + LUANCH_MAP_OFFSET_DEGREES);
    launchMap.put(3.5 + LAUNCH_MAP_OFFSET_M, 29.9 + LUANCH_MAP_OFFSET_DEGREES);
  }

  /** Returns the optimal angle given the robot's current pose */
  public static Rotation2d getLaunchMapAngle() {
    initializeLaunchMap();

    Rotation2d angle =
        Rotation2d.fromDegrees(launchMap.get(calculateSpeakerDistanceM().getAsDouble()));

    Logger.recordOutput("Shooter/TargetingSystem/Angle", angle);

    return angle;
  }

  /** Returns the optimal heading for shooting */
  public static Rotation2d getOptimalLaunchHeading() {
    double xDelta =
        (DriverStation.getAlliance().get() == Alliance.Blue)
            ? speakerOpeningBlue.getX() - currentRobotPose.get().getX()
            : speakerOpeningRed.getX() - currentRobotPose.get().getX();

    double yDelta =
        (DriverStation.getAlliance().get() == Alliance.Blue)
            ? speakerOpeningBlue.getY() - currentRobotPose.get().getY()
            : speakerOpeningRed.getY() - currentRobotPose.get().getY();

    Rotation2d heading = new Rotation2d(xDelta, yDelta);

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      heading = heading.plus(Rotation2d.fromDegrees(180.0));
    }

    Logger.recordOutput("Shooter/TargetingSystem/Heading", heading);

    return heading;
  }

  /** Update the current robot pose the targeting system uses */
  public static void updateRobotPose(Supplier<Pose2d> robotPose) {
    currentRobotPose = robotPose;
  }

  /** Calculate the tangental distance from the speaker */
  private static DoubleSupplier calculateSpeakerDistanceM() {
    double distanceM =
        (DriverStation.getAlliance().get() == Alliance.Blue)
            ? Math.hypot(
                speakerOpeningBlue.getX() - currentRobotPose.get().getX(),
                speakerOpeningBlue.getY() - currentRobotPose.get().getY())
            : Math.hypot(
                speakerOpeningRed.getX() - currentRobotPose.get().getX(),
                speakerOpeningRed.getY() - currentRobotPose.get().getY());

    Logger.recordOutput("Shooter/TargetingSystem/Distance", distanceM);

    return () -> distanceM;
  }
}
