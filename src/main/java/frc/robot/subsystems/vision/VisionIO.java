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
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    public double latencySeconds = 0.0;
    public boolean hasTarget = false;

    // LIMELIGHT ONLY (Since we're using object detection)
    public Pose2d cameraToObject =
        new Pose2d(); // Get the transform that maps camera space (X = forward, Y = left) to object
    // space (X forward, Y left)

    // PHOTON VISION ONLY (Since we're using apriltags)
    public Transform3d cameraToApriltag =
        new Transform3d(); // Get the transform that maps camera space (X = forward, Y = left, Z =
    // up) to fiducial tag space (X forward, Y left, Z up)
    public double poseAmbiguity = 0.0;
    public int aprilTagID = 0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
