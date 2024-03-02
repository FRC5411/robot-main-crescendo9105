// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  Debouncer debouncer;
  Pose3d offset;

  public VisionIOLimelight(String key, Pose3d offset, double debouncerTime) {
    this.offset = offset;

    table = NetworkTableInstance.getDefault().getTable(key);

    debouncer = new Debouncer(debouncerTime);
  }

  private Pose2d getTarget() {
    double[] posevalues = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    Translation2d translate =
        new Translation2d(posevalues[0] - offset.getX(), posevalues[1] - offset.getY());
    Rotation2d rotation =
        new Rotation2d(Math.toRadians(posevalues[3]) - offset.getRotation().getX());
    return new Pose2d(translate, rotation);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraToObject = getTarget();
    inputs.hasTarget = debouncer.calculate(table.getEntry("tv").getDouble(0.0) == 1);
    inputs.yaw = table.getEntry("tx").getDouble(0.0);
    inputs.pitch = table.getEntry("ty").getDouble(0.0);
    inputs.area = table.getEntry("ta").getDouble(0.0);
    inputs.latencySeconds =
        (table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0)) / 1000.0;
  }
}
