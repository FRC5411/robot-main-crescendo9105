// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json
// Apriltag map being used currently, can be cropped to only include the tags being used if
// necessary

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.FlexBuffers.Map;

public class Vision extends SubsystemBase {

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  private VisionIO cameraFrontLeft;
  private VisionIO cameraFrontRight;
  private VisionIO cameraSideLeft;
  private VisionIO cameraSideRight;

  private VisionIO[] io;
  private VisionIOInputs[] inputs;

  private final VisionIOInputsAutoLogged inputsFrontRight = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsFrontLeft = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsSideLeft = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsSideRight = new VisionIOInputsAutoLogged();

  public Vision(VisionIO cameraFrontLeft, VisionIO cameraFrontRight, VisionIO cameraSideLeft, VisionIO cameraSideRight) {
    io = new VisionIO[] {cameraFrontLeft, cameraFrontRight, cameraSideLeft, cameraSideRight};
    inputs = new VisionIOInputs[] {inputsFrontLeft, inputsFrontRight, inputsSideLeft, inputsSideRight};
  }


  public void setSingleStdDevs(double x, double y, double theta) {
    cameraFrontLeft.setSingleStdDevs(x, y, theta);
    cameraFrontRight.setSingleStdDevs(x, y, theta);
    cameraSideLeft.setSingleStdDevs(x, y, theta);
    cameraSideRight.setSingleStdDevs(x, y, theta);
  }

  public void setMultiStdDevs(double x, double y, double theta) {
    cameraFrontLeft.setMultiStdDevs(x, y, theta);
    cameraFrontRight.setMultiStdDevs(x, y, theta);
    cameraSideLeft.setMultiStdDevs(x, y, theta);
    cameraSideRight.setMultiStdDevs(x, y, theta);
  }

  public VisionIOInputsAutoLogged getInputsFrontRight() {
    return inputsFrontRight;
  }

  public VisionIOInputsAutoLogged getInputsFrontLeft() {
    return inputsFrontLeft;
  }

  public VisionIOInputsAutoLogged getInputsSideLeft() {
    return inputsSideLeft;
  }

  public VisionIOInputsAutoLogged getInputsSideRight() {
    return inputsSideRight;
  }

  public Matrix<N3, N1> getSingleStdDevsCoeff() {
    return cameraFrontLeft.getSingleStdDevsCoeff();
  }

  public Matrix<N3, N1> getMultiStdDevsCoeff() {
    return cameraFrontLeft.getMultiStdDevsCoeff();
  }
  
  @Override
  public void periodic() {
    // cameraFrontLeft.updateInputs(inputsFrontLeft);
    // cameraFrontRight.updateInputs(inputsFrontRight);
    // cameraSideLeft.updateInputs(inputsSideLeft);
    // cameraSideRight.updateInputs(inputsSideRight);

    // Logger.processInputs("Vision/FrontRight", inputsFrontRight);
    // Logger.processInputs("Vision/FrontLeft", inputsFrontLeft);
    // Logger.processInputs("Vision/SideLeft", inputsSideLeft);
    // Logger.processInputs("Vision/SideRight", inputsSideRight);
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<Vector<N3>> allVisionObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex] + timestampOffset.get();
        var values = inputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d =
                cameraPose.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
            useVisionRotation = true;
            break;
          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
              Rotation2d currentRotation =
                  RobotState.getInstance().().getRotation();
              Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose3d = robotPose3d0;
              } else {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              }
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }

        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose =
              aprilTagTypeSupplier.get().getLayout().getTagPose((int) values[i]);
          tagPose.ifPresent(tagPoses::add);
        }
        if (tagPoses.size() == 0) continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStdDev =
            xyStdDevCoefficient
                * Math.pow(avgDistance, 2.0)
                / tagPoses.size()
                * stdDevFactors[instanceIndex];
        double thetaStdDev =
            useVisionRotation
                ? thetaStdDevCoefficient
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.size()
                    * stdDevFactors[instanceIndex]
                : Double.POSITIVE_INFINITY;
        allVisionObservations.add(VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        // Log data from instance
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        //noinspection RedundantArrayCreation
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        aprilTagTypeSupplier
            .get()
            .getLayout()
            .getTagPose(detectionEntry.getKey())
            .ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));
  }
}
