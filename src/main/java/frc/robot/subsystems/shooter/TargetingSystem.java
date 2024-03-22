// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  private Translation3d speakerOpeningBlue = new Translation3d(0.0, 5.53, 1.045);
  private Translation3d speakerOpeningRed = new Translation3d(16.26, 5.53, 1.045);

  private final double LAUNCH_MAP_OFFSET_M = 0.93 + 0.46 - 0.23 - 0.17;
  private final double LUANCH_MAP_OFFSET_DEGREES_BLUE = 3.0 + 2.0; // 3.0;

  private final double LUANCH_MAP_OFFSET_DEG_AUTON_BLUE = -1.0;

  private final double LUANCH_MAP_OFFSET_DEGREES_RED = 1.0; // 4.0; // 3.0;

  private final double LUANCH_MAP_OFFSET_DEG_AUTON_RED = -0.0;

  private Drive robotDrive;
  private Vision robotVision;

  // @AutoLogOutput(key = "Shooter/TargetingSystem/MultiTagEnabled")
  private boolean multiTagEnabled = true;

  private Rotation2d lastHeading = new Rotation2d();

  private LinearFilter distanceFilter = LinearFilter.movingAverage(10);
  private LinearFilter rotationFilter = LinearFilter.movingAverage(10);

  private boolean useVision = true;

  private static TargetingSystem instance;

  /**
   * Tree Map that represents the robot's horizontal (X) distance from the Speaker (meters) and the
   * optimal launch angle (degrees)
   */
  private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

  public static TargetingSystem getInstance() {
    if (instance == null) {
      instance = new TargetingSystem();
      instance.initializeLaunchMap();
    }
    return instance;
  }

  public void setSubsystems(Drive drive, Vision vision) {
    robotDrive = drive;
    robotVision = vision;
  }

  /** Initialize the launch map */
  public void initializeLaunchMap() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        launchMap.put(0.0 + LAUNCH_MAP_OFFSET_M, 55.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(0.25 + LAUNCH_MAP_OFFSET_M, 52.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(0.50 + LAUNCH_MAP_OFFSET_M, 48.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(0.75 + LAUNCH_MAP_OFFSET_M, 45.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 42.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 40.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 38.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 37.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 35.6 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 34.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 33.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 32.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(3.0 + LAUNCH_MAP_OFFSET_M, 31.1 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(3.25 + LAUNCH_MAP_OFFSET_M, 30.3 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
        launchMap.put(3.5 + LAUNCH_MAP_OFFSET_M, 29.9 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      } else if (DriverStation.getAlliance().get() == Alliance.Red) {
        launchMap.put(0.0 + LAUNCH_MAP_OFFSET_M, 55.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(0.25 + LAUNCH_MAP_OFFSET_M, 52.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(0.50 + LAUNCH_MAP_OFFSET_M, 48.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(0.75 + LAUNCH_MAP_OFFSET_M, 45.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 42.5 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 40.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 38.5 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 37.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 35.6 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 34.5 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 33.5 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 32.0 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(3.0 + LAUNCH_MAP_OFFSET_M, 31.1 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(3.25 + LAUNCH_MAP_OFFSET_M, 30.3 + LUANCH_MAP_OFFSET_DEGREES_RED);
        launchMap.put(3.5 + LAUNCH_MAP_OFFSET_M, 29.9 + LUANCH_MAP_OFFSET_DEGREES_RED);
      }
    } else {
      launchMap.put(0.0 + LAUNCH_MAP_OFFSET_M, 55.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(0.25 + LAUNCH_MAP_OFFSET_M, 52.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(0.50 + LAUNCH_MAP_OFFSET_M, 48.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(0.75 + LAUNCH_MAP_OFFSET_M, 45.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 42.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 40.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 38.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 37.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.0 + LAUNCH_MAP_OFFSET_M, 35.6 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.25 + LAUNCH_MAP_OFFSET_M, 34.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.5 + LAUNCH_MAP_OFFSET_M, 33.5 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(1.75 + LAUNCH_MAP_OFFSET_M, 32.0 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(3.0 + LAUNCH_MAP_OFFSET_M, 31.1 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(3.25 + LAUNCH_MAP_OFFSET_M, 30.3 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
      launchMap.put(3.5 + LAUNCH_MAP_OFFSET_M, 29.9 + LUANCH_MAP_OFFSET_DEGREES_BLUE);
    }
  }

  /** Returns the optimal angle given the robot's current pose */
  public Rotation2d getLaunchMapAngle() {
    double distanceM = speakerDistanceM().getAsDouble();

    if (!multiTagEnabled) distanceM -= 0.4;
    Rotation2d angle = Rotation2d.fromDegrees(launchMap.get(distanceM));
    if (DriverStation.isAutonomous()) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Blue) {
        System.out.print("Angle:" + angle.getDegrees());
        angle = angle.plus(Rotation2d.fromDegrees(LUANCH_MAP_OFFSET_DEG_AUTON_BLUE));
        System.out.print("Angle Off:" + angle.getDegrees());
      } else {
        System.out.print("Angle:" + angle.getDegrees());
        angle = angle.plus(Rotation2d.fromDegrees(LUANCH_MAP_OFFSET_DEG_AUTON_RED));
        System.out.print("Angle Off:" + angle.getDegrees());
      }
    }

    Logger.recordOutput("Shooter/TargetingSystem/Angle", angle);

    if (angle == null) {
      if (distanceM < LAUNCH_MAP_OFFSET_M) {
        angle = Rotation2d.fromDegrees(27);
      } else if (distanceM > 3.5 + LAUNCH_MAP_OFFSET_M) {
        angle = Rotation2d.fromDegrees(57);
      }
    }

    return angle;
  }

  /** Returns the optimal heading for shooting */
  public Rotation2d getOptimalLaunchHeading() {
    Pose2d robotPose;
    if (useVision) robotPose = robotDrive.getFilteredPose();
    else robotPose = robotDrive.getOdometryPose();

    Rotation2d heading;
    if (multiTagEnabled) {
      double xDelta =
          (DriverStation.getAlliance().get() == Alliance.Blue)
              ? speakerOpeningBlue.getX() - robotPose.getX()
              : speakerOpeningRed.getX() - robotPose.getX();

      double yDelta =
          (DriverStation.getAlliance().get() == Alliance.Blue)
              ? speakerOpeningBlue.getY() - robotPose.getY()
              : speakerOpeningRed.getY() - robotPose.getY();

      heading = new Rotation2d(xDelta, yDelta);
    } else if (robotVision.getInputsLeft().hasSpeakerTarget) {
      heading = robotVision.getInputsLeft().speakerTagTransform.getTranslation().getAngle();
      lastHeading = heading;
    } else if (robotVision.getInputsRight().hasSpeakerTarget) {
      heading = robotVision.getInputsLeft().speakerTagTransform.getTranslation().getAngle();
      lastHeading = heading;
    } else {
      heading = lastHeading;
    }
    if (multiTagEnabled) {
      heading = heading.plus(Rotation2d.fromDegrees(180.0));
    } else {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        heading = heading.plus(Rotation2d.fromDegrees(180.0));
      }
    }

    Logger.recordOutput("Shooter/TargetingSystem/Heading", heading);

    if (!multiTagEnabled) {
      heading = Rotation2d.fromDegrees(rotationFilter.calculate(heading.getDegrees()));
    }

    return heading;
  }

  /** Calculate the tangental distance from the speaker */
  private double calculateSpeakerDistanceM() {
    Pose2d robotPose;
    if (useVision) robotPose = robotDrive.getFilteredPose();
    else robotPose = robotDrive.getOdometryPose();

    double distanceM;
    if (multiTagEnabled) {
      distanceM =
          (DriverStation.getAlliance().get() == Alliance.Blue)
              ? Math.hypot(
                  speakerOpeningBlue.getX() - robotPose.getX(),
                  speakerOpeningBlue.getY() - robotPose.getY())
              : Math.hypot(
                  speakerOpeningRed.getX() - robotPose.getX(),
                  speakerOpeningRed.getY() - robotPose.getY());
    } else if (robotVision.getInputsLeft().hasSpeakerTarget) {
      distanceM = robotVision.getInputsLeft().speakerTagTransform.getTranslation().getNorm();
    } else if (robotVision.getInputsRight().hasSpeakerTarget) {
      distanceM = robotVision.getInputsRight().speakerTagTransform.getTranslation().getNorm();
    } else {
      distanceM = -1;
    }

    Logger.recordOutput("Shooter/TargetingSystem/Distance", distanceM);

    distanceM = distanceFilter.calculate(distanceM);

    return distanceM;
  }

  private DoubleSupplier speakerDistanceM() {
    return () -> calculateSpeakerDistanceM();
  }

  public boolean isAtShootRange() {
    try {
      if (robotVision.getInputsLeft().hasSpeakerTarget
          || robotVision.getInputsRight().hasSpeakerTarget) {
        return false;
      }
      return speakerDistanceM().getAsDouble() <= 3;
    } catch (Exception e) {
      return false;
    }
  }

  public BooleanSupplier atShootRange() {
    return () -> isAtShootRange();
  }

  /** Returns a command to visualize a note being shot from the robot */
  public Command shoot(Supplier<Rotation2d> anglerPosition) {
    return new ScheduleCommand(
        Commands.defer(
                () -> {
                  Pose2d currentRobotPose = robotDrive.getFilteredPose();

                  final Transform3d anglerTransform =
                      new Transform3d(
                          0.105,
                          0.0,
                          0.232,
                          new Rotation3d(0.0, anglerPosition.get().getRadians(), 0.0));

                  final Pose3d startPose =
                      new Pose3d(
                              currentRobotPose.getX(),
                              currentRobotPose.getY(),
                              0.0,
                              new Rotation3d(
                                  0.0,
                                  0.0,
                                  robotDrive.getFilteredPose().getRotation().getRadians()))
                          .transformBy(anglerTransform);
                  final Pose3d endPose =
                      (DriverStation.getAlliance().get() == Alliance.Blue)
                          ? new Pose3d(speakerOpeningBlue, new Rotation3d())
                          : new Pose3d(speakerOpeningRed, new Rotation3d());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / 9.0;
                  final Timer timer = new Timer();

                  timer.start();

                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(() -> Logger.recordOutput("NoteVisualizer/ShotNotes"));
                },
                Set.of())
            .ignoringDisable(true));
  }

  public void toggleMultiTagEnabled() {
    multiTagEnabled = !multiTagEnabled;
  }

  public void toggleUseVision() {
    useVision = !useVision;
  }

  public void logMultiTagEnabled() {
    Logger.recordOutput("Shooter/TargetingSystem/MulitTagEnabled", multiTagEnabled);
  }

  public void logUseVision() {
    Logger.recordOutput("Shooter/TargetingSystem/UseVision", useVision);
  }

  public boolean getMultiTagEnabled() {
    return multiTagEnabled;
  }
}
