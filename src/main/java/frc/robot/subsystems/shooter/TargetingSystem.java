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
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class that calculates projectile motion given certain parameters */
public class TargetingSystem {
  private Translation3d speakerOpeningBlue = new Translation3d(0.0, 5.53, 1.045);
  private Translation3d speakerOpeningRed = new Translation3d(16.26, 5.53, 1.045);

  private final double LAUNCH_MAP_OFFSET_M = 0.99;

  private final double LAUNCH_MAP_OFFSET_DEGREES_BLUE = 5.0;
  private final double LAUNCH_MAP_OFFSET_DEG_AUTON_BLUE = -1.0;

  private final double LAUNCH_MAP_OFFSET_DEGREES_RED = 1.0;
  private final double LAUNCH_MAP_OFFSET_DEG_AUTON_RED = 0.0;

  private final int LINEAR_FILTER_DATASAMPLE = 10;

  private Drive robotDrive;
  private Vision robotVision;

  // @AutoLogOutput(key = "Shooter/TargetingSystem/MultiTagEnabled")
  private boolean multiTagEnabled = true;

  private Rotation2d lastHeading = new Rotation2d();

  private LinearFilter distanceFilter = LinearFilter.movingAverage(LINEAR_FILTER_DATASAMPLE);
  private LinearFilter rotationFilter = LinearFilter.movingAverage(LINEAR_FILTER_DATASAMPLE);

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

  public void initializeLaunchMap() {
    Alliance alliance = (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : null;
    double[][] shotMap = (alliance == Alliance.Red) ? ShooterConstants.redShotMap : ShooterConstants.blueShotMap;
    double launchMapOffsetDegrees = (alliance == Alliance.Red) ? LAUNCH_MAP_OFFSET_DEGREES_RED : LAUNCH_MAP_OFFSET_DEGREES_BLUE;
    
    for(int i = 0; i < shotMap.length; i++) {
      launchMap.put(
        shotMap[i][0] + LAUNCH_MAP_OFFSET_M, 
        shotMap[i][1] + launchMapOffsetDegrees);
    }
  }

  /** Returns the optimal angle given the robot's current pose */
  public Rotation2d getLaunchMapAngle() {
    double distanceM = speakerDistanceM().getAsDouble();

    if (!multiTagEnabled) distanceM -= 0.4;
    double mapAngle = launchMap.get(distanceM);
    if (launchMap == null) {
      mapAngle = (distanceM < LAUNCH_MAP_OFFSET_M) 
        ? 27
        : 57;
    }

    Rotation2d angle = Rotation2d.fromDegrees(mapAngle);
    if (DriverStation.isAutonomous()) {
      angle = angle.plus(Rotation2d.fromDegrees(
        (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) 
          ? LAUNCH_MAP_OFFSET_DEG_AUTON_RED : LAUNCH_MAP_OFFSET_DEG_AUTON_BLUE));
    }

    Logger.recordOutput("Shooter/TargetingSystem/Angle", angle);

    return angle;
  }

  public Rotation2d getOptimalLaunchHeading() {
    Pose2d robotPose = (useVision) ? robotDrive.getFilteredPose() : robotDrive.getOdometryPose();
    Rotation2d heading = new Rotation2d();

    if (multiTagEnabled) {
      double xDelta =
          (DriverStation.getAlliance().isPresent()) && 
          (DriverStation.getAlliance().get() == Alliance.Blue)
              ? speakerOpeningBlue.getX() - robotPose.getX()
              : speakerOpeningRed.getX() - robotPose.getX();

      double yDelta =
          (DriverStation.getAlliance().isPresent()) && 
          (DriverStation.getAlliance().get() == Alliance.Blue)
              ? speakerOpeningBlue.getY() - robotPose.getY()
              : speakerOpeningRed.getY() - robotPose.getY();

      heading = new Rotation2d(xDelta, yDelta);
    } else {
      VisionIOInputsAutoLogged inputs = 
            (robotVision.getInputsLeft().hasSpeakerTarget) ? robotVision.getInputsLeft() :
            (robotVision.getInputsRight().hasSpeakerTarget) ? robotVision.getInputsRight() :
            null;
      heading = (inputs != null) ? inputs.speakerTagTransform.getTranslation().getAngle() : lastHeading;
      lastHeading = heading;
    } 

    if(multiTagEnabled || (DriverStation.getAlliance().get() == Alliance.Red)) {
      heading = heading.plus(Rotation2d.fromDegrees(180.0));
    }
    
    if (!multiTagEnabled) {
      heading = Rotation2d.fromDegrees(rotationFilter.calculate(heading.getDegrees()));
    }

    Logger.recordOutput("Shooter/TargetingSystem/Heading", heading);

    return heading;
  }

  /** Calculate the tangental distance from the speaker */
  @AutoLogOutput(key = "Shooter/TargetingSystem/Speakerdistance")
  private double calculateSpeakerDistanceM() {
    Pose2d robotPose = (useVision) ? robotDrive.getFilteredPose() : robotDrive.getOdometryPose();
    double distanceM = 0;

    if (multiTagEnabled) {
      distanceM =
          ((DriverStation.getAlliance().isPresent()) && 
           (DriverStation.getAlliance().get() == Alliance.Blue))
              ? Math.hypot(
                  speakerOpeningBlue.getX() - robotPose.getX(),
                  speakerOpeningBlue.getY() - robotPose.getY())
              : Math.hypot(
                  speakerOpeningRed.getX() - robotPose.getX(),
                  speakerOpeningRed.getY() - robotPose.getY());
    } else {
      VisionIOInputsAutoLogged inputs = 
            (robotVision.getInputsLeft().hasSpeakerTarget) ? robotVision.getInputsLeft() :
            (robotVision.getInputsRight().hasSpeakerTarget) ? robotVision.getInputsRight() :
            null;
      distanceM = (inputs != null) ? inputs.speakerTagTransform.getTranslation().getNorm() : -1;
    }

    Logger.recordOutput("Shooter/TargetingSystem/Distance", distanceM);

    return distanceFilter.calculate(distanceM);
  }

  private DoubleSupplier speakerDistanceM() {
    return () -> calculateSpeakerDistanceM();
  }

  // public boolean isAtShootRange() {
  //   try {
  //     return (robotVision.getInputsLeft().hasSpeakerTarget || robotVision.getInputsRight().hasSpeakerTarget) 
  //               ? false 
  //               : speakerDistanceM().getAsDouble() <= 3;
  //   } catch (Exception e) {
  //     return false;
  //   }
  // }

  // public BooleanSupplier atShootRange() {
  //   return () -> isAtShootRange();
  // }

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

  // public void toggleMultiTagEnabled() {
  //   multiTagEnabled = !multiTagEnabled;
  // }

  // public void toggleUseVision() {
  //   useVision = !useVision;
  // }

  public void logMultiTagEnabled() {
    Logger.recordOutput("Shooter/TargetingSystem/MulitTagEnabled", multiTagEnabled);
  }

  public void logUseVision() {
    Logger.recordOutput("Shooter/TargetingSystem/UseVision", useVision);
  }

  // public boolean getMultiTagEnabled() {
  //   return multiTagEnabled;
  // }
}