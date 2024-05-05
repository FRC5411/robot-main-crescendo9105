// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonSim implements VisionIO {
  private PhotonCamera limelightCam;
  private PhotonPoseEstimator poseEstimator;
  private Transform3d cameraTransform;
  private Debouncer debouncer;

  private PhotonCameraSim limelightSim;
  private VisionSystemSim visionSim;
  private Supplier<Pose2d> drivePose;

  private int speakerTagID =
      (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
              == DriverStation.Alliance.Red)
          ? 3
          : 7;

  public VisionIOPhotonSim(
      String name, Transform3d cameraTransform, double debouncerTime, Supplier<Pose2d> drivePose) {
    limelightCam = new PhotonCamera(name);
    PhotonCamera.setVersionCheckEnabled(false);
    this.cameraTransform = cameraTransform;
    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            limelightCam,
            cameraTransform);

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    debouncer = new Debouncer(debouncerTime);

    // Create the vision system simulation which handles cameras and targets on the field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    visionSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    // Create simulated camera properties. These can be set to mimic your actual camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(75));
    cameraProp.setCalibError(0.51, 0.28);
    cameraProp.setFPS(100);
    cameraProp.setAvgLatencyMs(90);
    cameraProp.setLatencyStdDevMs(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    limelightSim = new PhotonCameraSim(limelightCam, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(limelightSim, cameraTransform);

    limelightSim.enableDrawWireframe(true);

    this.drivePose = drivePose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(drivePose.get());

    PhotonPipelineResult result = limelightCam.getLatestResult();
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);

    inputs.isConnected = limelightCam.isConnected();

    inputs.hasTarget = result.hasTargets();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      inputs.hasTargetDebounced = debouncer.calculate(inputs.hasTarget);
      inputs.cameraToApriltag = target.getBestCameraToTarget();
      inputs.robotToApriltag = target.getBestCameraToTarget().plus(cameraTransform);
      inputs.aprilTagID = target.getFiducialId();
      inputs.poseAmbiguity = target.getPoseAmbiguity();
      inputs.yaw = target.getYaw();
      inputs.pitch = target.getPitch();
      inputs.area = target.getArea();
      inputs.latencySeconds = result.getLatencyMillis() / 1000.0;

      inputs.numberOfTargets = getApriltagCount(result);

      inputs.hasSpeakerTarget = poseEstimator.getFieldTags().getTagPose(speakerTagID).isPresent();
    }

    inputs.latestTimestamp = result.getTimestampSeconds();

    estimatedRobotPose.ifPresent(
        est -> {
          inputs.estimatedRobotPose = estimatedRobotPose.get().estimatedPose.toPose2d();

          visionSim.getDebugField().setRobotPose(inputs.estimatedRobotPose);

          inputs.averageDistanceFromTagMeters = averageDistance(
            inputs.estimatedRobotPose, result, inputs.numberOfTargets);
        });
  }

  private int getApriltagCount(PhotonPipelineResult result) {
    List<PhotonTrackedTarget> targets = result.getTargets();
    int numTags = 0;

    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) continue;

      // Increase number of tags
      numTags++;
    }

    return numTags;
  }

  private double averageDistance(Pose2d estimatedPose, PhotonPipelineResult result, int numTags) {
    List<PhotonTrackedTarget> targets = result.getTargets();
    double avgDist = 0;

    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) continue;

      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    avgDist /= numTags;

    return avgDist;
  }
}
