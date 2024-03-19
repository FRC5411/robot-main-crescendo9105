// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for gyros */
public interface GyroIO {
  /** Drive subsystem gyro sensor data */
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadiansPerSecond = 0.0;
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Reset the gyro yaw */
  public default void resetGyro() {}
}
