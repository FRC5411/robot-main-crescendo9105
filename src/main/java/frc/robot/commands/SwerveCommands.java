// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/** Class to hold all of the commands for the Drive */
public class SwerveCommands {
  private static final double DEADBAND = 0.1;
  private static final boolean IS_FIELD = true;

  private SwerveCommands() {}

  /** Command to drive the swerve with joysticks | Field-relative */
  public static Command swerveDrive(
      Drive robotDrive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier) {
    return Commands.run(
        () -> {
          // Forward, backward
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          // Left, right
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          // Rotation
          double theta = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), DEADBAND);

          // Square inputs
          linearMagnitude *= linearMagnitude;
          theta = Math.copySign(theta * theta, theta);

          // Calculate velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          if (IS_FIELD) {
            robotDrive.runSwerve(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    // Convert from % to MPS
                    linearVelocity.getX() * robotDrive.getMaxLinearSpeedMPS(),
                    linearVelocity.getY() * robotDrive.getMaxLinearSpeedMPS(),
                    theta * robotDrive.getMaxAngularSpeedMPS(),
                    robotDrive.getRotation()));
          } else {
            robotDrive.runSwerve(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    linearVelocity.getX() * robotDrive.getMaxLinearSpeedMPS(),
                    linearVelocity.getY() * robotDrive.getMaxLinearSpeedMPS(),
                    theta * robotDrive.getMaxAngularSpeedMPS(),
                    robotDrive.getRotation()));
          }
        },
        robotDrive);
  }
}
