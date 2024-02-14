package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraIOPhotonVision implements CameraIO {
  String cameraName;
  int pipeline;
  PhotonCamera camera;
  PhotonPoseEstimator poseEstimator;

  public CameraIOPhotonVision(String cameraName) {
    this.cameraName = cameraName;
    this.pipeline = 0;
    this.camera = new PhotonCamera(this.cameraName);
    this.poseEstimator = new PhotonPoseEstimator(
        AprilTagFields.k2024Crescendo, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        camera,
        new Transform3d());
  }

  public void setPipeline(int pipeline) {
    this.pipeline = pipeline;
  }

  public void updateInputs(CameraIOLoggedInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();

    inputs.cameraName = cameraName;
    inputs.pipeline = pipeline;
    inputs.timeStamp = result.getTimestampSeconds();

    inputs.hasTarget = result.hasTargets();
    inputs.targetType = "Apriltag";
    if(result.hasTargets()) {
        inputs.targetYaw = result.getBestTarget().getYaw();
        inputs.targetRoll = result.getBestTarget().getPitch();
        inputs.targetPitch = result.getBestTarget().getPitch();
        inputs.amountOfTargets = result.getTargets().size();
        inputs.targetArea = result.getBestTarget().getArea();

        inputs.targetPose = result.getMultiTagResult().estimatedPose.best;
    }
  }
}
