// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double[] demoFrame = new double[] {};
    public long fps = 0;

    public boolean isConnected = false;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    public double latencySeconds = 0.0;
    public boolean hasTarget = false;
    public boolean hasTargetDebounced = false;
    public int numberOfTargets = 0;

    // LIMELIGHT ONLY (Since we're using object detection)
    public Pose2d cameraToObject = new Pose2d();

    // PHOTON VISION ONLY (Since we're using apriltags)
    public Transform3d cameraToApriltag = new Transform3d();
    public double poseAmbiguity = 0.0;
    public int aprilTagID = 0;
    public Transform3d robotToApriltag = new Transform3d();
    public double latestTimestamp = 0.0;
    public Pose2d estimatedRobotPose = new Pose2d();
    public double xStandardDeviation = 0.0;
    public double yStandardDeviation = 0.0;
    public double thetaStandardDeviation = 0.0;

    public Pose2d speakerTagPose = new Pose2d();
    public Transform2d speakerTagTransform = new Transform2d();

    public boolean hasSpeakerTarget = false;

    public double speakerXStdDev = 0.0;
    public double speakerYStdDev = 0.0;
    public double speakerThetaDev = 0.0;


  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setSingleStdDevs(double x, double y, double theta) {}

  public default void setMultiStdDevs(double x, double y, double theta) {}

  public default Matrix<N3, N1> getSingleStdDevsCoeff() {
    return VecBuilder.fill(0, 0, 0);
  }

  public default Matrix<N3, N1> getMultiStdDevsCoeff() {
    return VecBuilder.fill(0, 0, 0);
  }
}
