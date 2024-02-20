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
   * Units: Meters. When facing the field from the source: X is the left-right, Y is the front-back,
   * Z is the up-down
   */
  private static Translation3d speakerOpening = new Translation3d(0.26, 5.49, 2.045);

  private static Pose2d currentPose = new Pose2d();
  private static Rotation2d currentAnglerRotation = new Rotation2d();
  private static double initialVelocityMPS = 0.0;

  private static double distanceFromWallMeters = 0.0;

  /**
   * Calculate the trajectory of shooting a gamepice (motionless-robot) without the initial Shooter
   * angle
   */
  public static Pose3d[] calculateTrajectory(Pose2d robotPose, double launchVelocityMPS) {
    currentPose = robotPose;
    initialVelocityMPS = launchVelocityMPS;
    currentAnglerRotation = new Rotation2d();

    currentAnglerRotation = guessAngle(currentAnglerRotation, launchVelocityMPS, 1e-4);

    return calculateTrajectory(currentPose, currentAnglerRotation, launchVelocityMPS);
  }

  private static Rotation2d guessAngle(
      Rotation2d anglerRotation, double launchVelocityMPS, double deltaTheta) {
    double epsilon = 0.0004;

    for (int i = 0; i < 1000; i++) {
      Rotation2d thetaOne = currentAnglerRotation;
      Rotation2d thetaTwo = currentAnglerRotation.plus(new Rotation2d(epsilon));

      double derivativeOfTrajectory =
          (calculateTrajectory(currentPose, thetaTwo, launchVelocityMPS)[1].getZ()
                  - calculateTrajectory(currentPose, thetaOne, launchVelocityMPS)[1].getZ())
              / (thetaOne.minus(thetaTwo).getRadians());
      // (calculateTrajectory(currentPose, thetaTwo, launchVelocityMPS)[1].getZ()
      //         - calculateTrajectory(currentPose, thetaOne, launchVelocityMPS)[1].getZ())
      //     / epsilon;

      thetaOne =
          thetaOne.minus(
              Rotation2d.fromRadians(
                  calculateTrajectory(currentPose, thetaOne, launchVelocityMPS)[1].getZ()
                      / derivativeOfTrajectory));
      // newTheta = newTheta.times(-1.0);
      System.out.println(i + " Theta: " + thetaOne);

      if (Math.abs(thetaOne.getRadians() - currentAnglerRotation.getRadians()) < 1e-6) {
        return thetaOne;
      }

      currentAnglerRotation = thetaOne;
    }
    return currentAnglerRotation;
  }

  /** Calculate the trajectory of shooting a gamepiece (motionless-robot) */
  public static Pose3d[] calculateTrajectory(
      Pose2d robotPose, Rotation2d anglerRotation, double launchVelocityMPS) {
    // List of poses to represent the path of the note (start, end)
    Pose3d[] trajectory = new Pose3d[2];
    // TODO ------------------ Recalculate height offset when shooter angle changes
    // First pose is the current pose of the robot as well as the initial launcher height
    trajectory[0] = new Pose3d(robotPose.getX(), robotPose.getY(), 0.25, new Rotation3d());

    // Final x-position of the projectile
    distanceFromWallMeters = (speakerOpening.getX() - (speakerOpening.getX() - robotPose.getX()));

    // Calculate initial trajectory path (no gravity)
    double baseEstimatedProjectileZ =
        distanceFromWallMeters * Math.tan(anglerRotation.getRadians());

    // Calculate the effect of gravity
    double estimatedEffectOfGravity =
        (9.81 * Math.pow(distanceFromWallMeters, 2.0))
            / ((2.0 * Math.pow(launchVelocityMPS, 2.0))
                * (Math.pow(Math.cos(anglerRotation.getRadians()), 2.0)));

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

  /** Get the derivitive of a calculated trajectory */
  private static double derivativeOfTrajectory(
      double initialVelocityMPS, Rotation2d anglerRotation, double deltaTheta) {
    Rotation2d thetaOne = anglerRotation;
    Rotation2d thetaTwo = Rotation2d.fromRadians(anglerRotation.getRadians() + deltaTheta);

    double derivative =
        (calculateTrajectory(currentPose, thetaTwo, initialVelocityMPS)[1].getZ()
                - calculateTrajectory(currentPose, thetaOne, initialVelocityMPS)[1].getZ())
            / deltaTheta;

    System.out.println(
        "\033[93mTrajOne:\033[0m "
            + calculateTrajectory(currentPose, thetaOne, initialVelocityMPS)[1].getZ());
    System.out.println(
        "\033[93mTrajTwo:\033[0m "
            + calculateTrajectory(currentPose, thetaTwo, initialVelocityMPS)[1].getZ());
    System.out.println("\033[93mDerivative:\033[0m " + derivative);

    return derivative;
  }

  /** Get the tangent line of the derivative of the trajectory endpoint */
  private static double tangentOfTrajectoryDerivative(
      double theta, double trajectoryZ, double derivative) {
    final double TANGENT_LINE = (trajectoryZ / derivative) + theta;

    System.out.println("\033[93mTheta:\033[0m " + theta);
    System.out.println("\033[93mTangent:\033[0m " + TANGENT_LINE);

    return TANGENT_LINE;
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

    Pose3d[] estimatedTrajectory =
        calculateTrajectory(new Pose2d(3.0, speakerOpening.getY(), new Rotation2d()), 15.16);
    Logger.recordOutput("Shooter/Generator/EstimatedTrajectory", estimatedTrajectory);
  }

  // int i = 0;
  // // while (Math.abs(
  // //             calculateTrajectory(currentPose, currentAnglerRotation, initialVelocityMPS)[1]
  // //                 .getZ())
  // //         > Units.degreesToRadians(1e-4)
  // //     && i < 100) {
  // while (!(calculateTrajectory(currentPose, currentAnglerRotation,
  // initialVelocityMPS)[1].getZ()
  //             <= 2.05
  //         && calculateTrajectory(currentPose, currentAnglerRotation,
  // initialVelocityMPS)[1].getZ()
  //             >= 2.0)
  //     && i < 100) {

  //   double estimatedTrajectory =
  //       calculateTrajectory(currentPose, currentAnglerRotation, initialVelocityMPS)[1].getZ();
  //   // System.out.println("\033[93mEstmated:\033[0m " + estimatedTrajectory);
  //   double derivative = derivativeOfTrajectory(initialVelocityMPS, currentAnglerRotation,
  // 1e-4);

  //   currentAnglerRotation =
  //       new Rotation2d(
  //           tangentOfTrajectoryDerivative(
  //               currentAnglerRotation.getRadians(), estimatedTrajectory, derivative));

  //   if (i < 10) {
  //     System.out.println("00" + i + " Shooter/Generator/Debug/Theta " + currentAnglerRotation);

  //   } else if (i < 100) {
  //     System.out.println("0" + i + " Shooter/Generator/Debug/Theta " + currentAnglerRotation);
  //   } else {
  //     System.out.println(i + " Shooter/Generator/Debug/Theta " + currentAnglerRotation);
  //   }
  //   System.out.println("+-------------------------------+");

  //   Logger.recordOutput("Shooter/Generator/Debug/Theta", currentAnglerRotation.getRadians());
  //   Logger.recordOutput("Shooter/Generator/Debug/Derivative", derivative);
  //   Logger.recordOutput("Shooter/Generator/Debug/TrajectoryZ", estimatedTrajectory);
  //   Logger.recordOutput("Shooter/Generator/Debug/Iteration", i);

  //   i++;
  // }
}
