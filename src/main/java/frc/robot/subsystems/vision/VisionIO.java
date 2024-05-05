// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean isConnected = false;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    public double latencySeconds = 0.0;
    public boolean hasTarget = false;
    public boolean hasTargetDebounced = false;
    public int numberOfTargets = 0;

    // PHOTON VISION ONLY (Since we're using apriltags)
    public Transform3d cameraToApriltag = new Transform3d();
    public double poseAmbiguity = 0.0;
    public int aprilTagID = 0;
    public Transform3d robotToApriltag = new Transform3d();
    public double latestTimestamp = 0.0;
    public Pose2d estimatedRobotPose = new Pose2d();
    public double averageDistanceFromTagMeters = 0.0;

    public boolean hasSpeakerTarget = false;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
