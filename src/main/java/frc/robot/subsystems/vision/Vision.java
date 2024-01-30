package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;
import frc.robot.subsystems.vision.VisionConstants.LL;

public class Vision extends SubsystemBase {
  private Limelight centerLimeLight;
  private Limelight leftLimelight;
  private Limelight rightLimelight;
  private Limelight[] limelights;

  public Vision() {
    centerLimeLight = new Limelight(LL.centerLLNT, new Pose3d(), 0.2);
    leftLimelight = new Limelight(LL.leftLLNT, LL.leftLLOffsetMeters, 0.4);
    rightLimelight = new Limelight(LL.rightLLNT, LL.rightLLOffsetMeters, 0.4);

    centerLimeLight.setPipelineIndex(LL.apriltagPipelineIndex);

    leftLimelight.setPipelineIndex(LL.apriltagPipelineIndex);
    rightLimelight.setPipelineIndex(LL.apriltagPipelineIndex);

    limelights = new Limelight[] {leftLimelight, rightLimelight, centerLimeLight};
  }

  public Limelight getcenterLimeLight() {
    return centerLimeLight;
  }

  public Limelight getLeftLimelight() {
    return leftLimelight;
  }

  public Limelight getRightLimelight() {
    return rightLimelight;
  }

  public double getNorm() {
    return getcenterLimeLight().getTarget().getTranslation().getNorm();
  }

  public void setPipelineIndices(int index) {
    for (Limelight cam : limelights) cam.setPipelineIndex(index);
  }

  public void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator) {
    if (centerLimeLight.hasTargetDebounced()
        && centerLimeLight.getPipeLineIndex() == LL.apriltagPipelineIndex) {
      System.out.println("BACK Limelight TIME: " + Timer.getFPGATimestamp());
      poseEstimator.addVisionMeasurement(
          centerLimeLight.getPose(),
          Timer.getFPGATimestamp() - centerLimeLight.getLatency(),
          createVisionVector(centerLimeLight));
    }

    if (leftLimelight.hasTargetDebounced()
        && leftLimelight.getPipeLineIndex() == LL.apriltagPipelineIndex) {
      System.out.println("LEFT Limelight TIME: " + Timer.getFPGATimestamp());
      poseEstimator.addVisionMeasurement(
          leftLimelight.getPose(),
          Timer.getFPGATimestamp() - leftLimelight.getLatency(),
          createVisionVector(leftLimelight));
    }

    if (rightLimelight.hasTargetDebounced()
        && rightLimelight.getPipeLineIndex() == LL.apriltagPipelineIndex) {
      System.out.println("RIGHT Limelight TIME: " + Timer.getFPGATimestamp());
      poseEstimator.addVisionMeasurement(
          rightLimelight.getPose(),
          Timer.getFPGATimestamp() - rightLimelight.getLatency(),
          createVisionVector(rightLimelight));
    }
  }

  public Vector<N3> createVisionVector(Limelight cam) {
    final double kStdDev = 3.8;
    final int kStdDevLast = 10000000;
    return VecBuilder.fill(kStdDev * getNorm(), kStdDev * getNorm(), kStdDevLast);
  }

  @Override
  public void periodic() {
    centerLimeLight.periodic();
    leftLimelight.periodic();
    rightLimelight.periodic();

    // TODO: Add these to AK
    centerLimeLight.getPose().minus(leftLimelight.getPose()).getTranslation().getX();
    centerLimeLight.getPose().minus(rightLimelight.getPose()).getTranslation().getX();
    centerLimeLight.getPose().minus(leftLimelight.getPose()).getTranslation().getY();
    centerLimeLight.getPose().minus(rightLimelight.getPose()).getTranslation().getY();
  }
}
