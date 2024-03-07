// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  PhotonCamera frontLimelight;
  PhotonPoseEstimator poseEstimator;
  Transform3d cameraTransform;
  Matrix<N3, N1> singleTagStdDevs;
  Matrix<N3, N1> multiTagStdDevs;
  Debouncer debouncer;

  public VisionIOPhotonVision(String name, Transform3d cameraTransform, double debouncerTime) {
    AprilTagFields tagFields = AprilTagFields.k2024Crescendo;

    singleTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    multiTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    frontLimelight = new PhotonCamera(name);
    PhotonCamera.setVersionCheckEnabled(false);
    this.cameraTransform = cameraTransform;
    poseEstimator =
        new PhotonPoseEstimator(
            tagFields.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontLimelight,
            cameraTransform);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    debouncer = new Debouncer(debouncerTime);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = frontLimelight.getLatestResult();
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);

    inputs.hasTarget = result.hasTargets();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      inputs.hasTargetDebounced = debouncer.calculate(inputs.hasTarget);
      inputs.cameraToApriltag = target.getBestCameraToTarget();
      inputs.robotToApriltag = inputs.cameraToApriltag.plus(cameraTransform);
      inputs.aprilTagID = target.getFiducialId();
      inputs.poseAmbiguity = target.getPoseAmbiguity();
      inputs.yaw = target.getYaw();
      inputs.pitch = target.getPitch();
      inputs.area = target.getArea();
      inputs.latencySeconds = result.getLatencyMillis() / 1000.0;

      inputs.numberOfTargets = getApriltagCount(result);
    }

    inputs.latestTimestamp = result.getTimestampSeconds();

    estimatedRobotPose.ifPresent(
        est -> {
          inputs.estimatedRobotPose = estimatedRobotPose.get().estimatedPose.toPose2d();

          Matrix<N3, N1> standardDevs = getEstimationStdDevs(inputs.estimatedRobotPose, result);

          inputs.xStandardDeviation = standardDevs.get(0, 0);
          inputs.yStandardDeviation = standardDevs.get(1, 0);
          inputs.thetaStandardDeviation = standardDevs.get(2, 0);
        });
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
    List<PhotonTrackedTarget> targets = result.getTargets();
    Matrix<N3, N1> estStdDevs = singleTagStdDevs;

    int numTags = 0;
    double avgDist = 0;

    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) continue;

      // Increase number of tags
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    // No tags visible
    if (numTags == 0) return estStdDevs;

    // Calculate average distance
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = multiTagStdDevs;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public int getApriltagCount(PhotonPipelineResult result) {
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
}