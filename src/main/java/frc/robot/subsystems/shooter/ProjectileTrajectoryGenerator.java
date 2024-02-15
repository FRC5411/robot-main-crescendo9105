// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

/** Object that can calculate the optimal shooter subsystem angle and heading for the goal */
public class ProjectileTrajectoryGenerator {
  /**
   * In meters--When facing the field from the source: X is the left-right, Y is the front-back, Z
   * is the up-down
   */
  private static Translation3d speakerOpening = new Translation3d(0.26, 5.49, 2.045);

  private static Pose2d currentPose = new Pose2d();
  private static Rotation2d currentAnglerRotation = new Rotation2d();
  private static double initialVelocityMPS = 0.0;

  /** Calculate the trajectory of shooting a gamepiece (motionless-robot) */
  public static Pose3d[] calculateTrajectory(
      Pose2d robotPose, Rotation2d anglerRotation, double launchVelocityMPS) {
    currentPose = robotPose;
    currentAnglerRotation = anglerRotation;
    initialVelocityMPS = launchVelocityMPS;

    // List of poses to represent the path of the note (start, end)
    Pose3d[] trajectory = new Pose3d[2];
    // TODO ------------------ Recalculate height offset when shooter angle changes ------------------
    // First pose is the current pose of the robot as well as the initial launcher height
    trajectory[0] = new Pose3d(currentPose.getX(), currentPose.getY(), 0.25, new Rotation3d());

    // Final x-position of the projectile
    final double DISTANCE_X =
        (speakerOpening.getX() - (speakerOpening.getX() - currentPose.getX()));

    // Calculate initial trajectory path (no gravity)
    double baseEstimatedProjectileZ = DISTANCE_X * Math.tan(currentAnglerRotation.getRadians());

    // Calculate the effect of gravity
    double estimatedEffectOfGravity =
        (9.81 * Math.pow(DISTANCE_X, 2.0))
            / ((2.0 * Math.pow(initialVelocityMPS, 2.0))
                * (Math.pow(Math.cos(currentAnglerRotation.getRadians()), 2.0)));

    // Get the final height of the projectile accounting for initial offsets and gravity
    final double CALCULATED_PROJECTILE_Z =
        baseEstimatedProjectileZ - estimatedEffectOfGravity + trajectory[0].getZ();
    trajectory[1] =
        new Pose3d(
            speakerOpening.getX(),
            speakerOpening.getY(),
            CALCULATED_PROJECTILE_Z,
            new Rotation3d());

    return trajectory;
  }

  // TODO Remove after development stage
  /** Display information for debugging */
  public static void displayDebuggingInformation() {
    Logger.recordOutput(
        "Shooter/Generator/SpeakerOpening", new Pose3d(speakerOpening, new Rotation3d()));

    Pose3d[] calculatedTrajectory =
        calculateTrajectory(
            new Pose2d(3.0, speakerOpening.getY(), new Rotation2d()),
            Rotation2d.fromDegrees(36.5),
            15.16);

    Logger.recordOutput("Shooter/Generator/CalculatedTrajectory", calculatedTrajectory);
  }
}
