// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  PhotonCamera frontLimelight;

  public VisionIOPhotonVision(String name) {
    frontLimelight = new PhotonCamera(name);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = frontLimelight.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    inputs.hasTarget = result.hasTargets();
    inputs.cameraToApriltag = target.getBestCameraToTarget();
    inputs.aprilTagID = target.getFiducialId();
    inputs.poseAmbiguity = target.getPoseAmbiguity();
    inputs.yaw = target.getYaw();
    inputs.pitch = target.getPitch();
    inputs.area = target.getArea();
    inputs.latencySeconds = result.getLatencyMillis() / 1000.0;
  }
}
