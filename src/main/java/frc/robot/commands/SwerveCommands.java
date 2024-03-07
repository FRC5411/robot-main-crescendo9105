// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Class to hold all of the commands for the Drive */
public class SwerveCommands {
  private static final double DEADBAND = 0.1;
  private static final boolean IS_FIELD = true;

  private static Command currentCommand = null;

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
                    robotDrive.getPosition().getRotation()));
          } else {
            robotDrive.runSwerve(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    linearVelocity.getX() * robotDrive.getMaxLinearSpeedMPS(),
                    linearVelocity.getY() * robotDrive.getMaxLinearSpeedMPS(),
                    theta * robotDrive.getMaxAngularSpeedMPS(),
                    robotDrive.getPosition().getRotation()));
          }
        },
        robotDrive);
  }

  /*
  *         () -> {

         Rotation2d goalRotation = thetaGoal.get();

         Rotation2d currentRotation = robotDrive.getPosition().getRotation();

         if (Math.abs(goalRotation.getDegrees() - currentRotation.getDegrees()) > 5) {
           driveThetaController.setGoal(currentRotation.getDegrees() + 180);
         }
       },
       () -> {
         double desiredXSpeed = driveXLimiter.calculate(xGoal.get());
         double desiredYSpeed = driveYLimiter.calculate(yGoal.get());

         double desiredThetaDegrees =
             driveThetaController.calculate(
                 robotDrive.getPosition().getRotation().getDegrees(),
                 thetaGoal.get().getDegrees());
  */
  public static Command setHeading(
      Drive robotDrive,
      DoubleSupplier xGoalSupplier,
      DoubleSupplier yGoalSupplier,
      Supplier<Rotation2d> headingGoalSupplier) {
    SlewRateLimiter xSpeedsLimiter = new SlewRateLimiter(5.0);
    SlewRateLimiter ySpeedsLimiter = new SlewRateLimiter(5.0);

    // Degress per second
    ProfiledPIDController thetaFeedback =
        switch (Constants.currentMode) {
          case REAL -> new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(300.0, 200.0));
          case SIM -> new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(300.0, 200.0));
          default -> new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
        };

    thetaFeedback.setTolerance(0.2);
    thetaFeedback.enableContinuousInput(0.0, 360.0);

    LoggedTunableNumber thetaFeedbackP =
        new LoggedTunableNumber("Drive/HeadingController/Feedback/P", thetaFeedback.getP());
    LoggedTunableNumber thetaFeedbackI =
        new LoggedTunableNumber("Drive/HeadingController/Feedback/I", thetaFeedback.getI());
    LoggedTunableNumber thetaFeedbackD =
        new LoggedTunableNumber("Drive/HeadingController/Feedback/D", thetaFeedback.getD());
    LoggedTunableNumber thetaFeedbackV =
        new LoggedTunableNumber(
            "Drive/HeadingController/Feedback/V", thetaFeedback.getConstraints().maxVelocity);
    LoggedTunableNumber thetaFeedbackA =
        new LoggedTunableNumber(
            "Drive/HeadingController/Feedback/A", thetaFeedback.getConstraints().maxAcceleration);

    currentCommand =
        new FunctionalCommand(
            () -> {
              xSpeedsLimiter.reset(robotDrive.getDesiredChassisSpeeds().vxMetersPerSecond);
              ySpeedsLimiter.reset(robotDrive.getDesiredChassisSpeeds().vyMetersPerSecond);

              Rotation2d headingGoal = headingGoalSupplier.get();
              Rotation2d currentHeading = robotDrive.getRotation();

              // If the error is small, set the goal to be the current heading
              if (Math.abs(headingGoal.getDegrees() - currentHeading.getDegrees()) > 5.0) {
                // Add 180 since front is the intake, not the shooter
                thetaFeedback.setGoal(currentHeading.getDegrees());
              }

              thetaFeedback.setP(thetaFeedbackP.get());
              thetaFeedback.setI(thetaFeedbackI.get());
              thetaFeedback.setD(thetaFeedbackD.get());
              thetaFeedback.setConstraints(
                  new Constraints(thetaFeedbackV.get(), thetaFeedbackA.get()));

              Logger.recordOutput("Drive/HeadingController/Error", 0.0);
              Logger.recordOutput("Drive/HeadingController/Setpoint", 0.0);
              Logger.recordOutput("Drive/HeadingController/Goal", 0.0);
              Logger.recordOutput("Drive/HeadingController/AtGoal", false);
              Logger.recordOutput("Drive/HeadingController/Output", 0.0);
            },
            () -> {
              double xDesiredSpeedMPS = xSpeedsLimiter.calculate(xGoalSupplier.getAsDouble());
              double yDesiredSpeedMPS = ySpeedsLimiter.calculate(yGoalSupplier.getAsDouble());

              double thetaDesiredDegrees =
                  thetaFeedback.calculate(
                      robotDrive.getPosition().getRotation().getDegrees(),
                      headingGoalSupplier.get().getDegrees());

              robotDrive.runSwerve(
                  new ChassisSpeeds(
                      xDesiredSpeedMPS, yDesiredSpeedMPS, Math.toRadians(thetaDesiredDegrees)));

              Logger.recordOutput(
                  "Drive/HeadingController/Error", thetaFeedback.getPositionError());
              Logger.recordOutput(
                  "Drive/HeadingController/Setpoint", thetaFeedback.getSetpoint().position);
              Logger.recordOutput("Drive/HeadingController/Goal", thetaFeedback.getGoal().position);
              Logger.recordOutput("Drive/HeadingController/AtGoal", thetaFeedback.atGoal());
              Logger.recordOutput("Drive/HeadingController/Output", thetaDesiredDegrees);
            },
            (interrupted) -> {
              robotDrive.stop();
            },
            () -> thetaFeedback.atGoal(),
            robotDrive);

    return currentCommand;
  }

  /** Returns a command to reset the gyro heading */
  public static Command resetGyro(Drive robotDrive) {
    currentCommand = Commands.runOnce(() -> robotDrive.resetGyro(), robotDrive);

    return currentCommand;
  }

  /** Returns a command to set the robot pose */
  public static Command setPose(Drive robotDrive, Pose2d desiredPose) {
    currentCommand = Commands.runOnce(() -> robotDrive.setPose(desiredPose), robotDrive);

    return currentCommand;
  }

  /** Returns a command to stop the drivetrain */
  public static Command stopDrive(Drive robotDrive) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }

    currentCommand = Commands.runOnce(() -> robotDrive.stop(), robotDrive);

    return currentCommand;
  }
}
