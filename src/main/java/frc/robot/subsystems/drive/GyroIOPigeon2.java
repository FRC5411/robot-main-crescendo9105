// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** A Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private Pigeon2 gyro = new Pigeon2(10);

  private StatusSignal<Double> yaw = gyro.getYaw();
  private StatusSignal<Double> yawVelocity = gyro.getAngularVelocityZWorld();

  private Queue<Double> odometryTimestampQueue;
  private Queue<Double> yawPositionQueue;

  public GyroIOPigeon2(boolean phoenixDrive) {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);

    gyro.optimizeBusUtilization();

    // Add yaw odometry to different queues based on drivebase type
    if (phoenixDrive) {
      odometryTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro, gyro.getYaw());
    } else {
      odometryTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(() -> gyro.getYaw().getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Refresh telemetry signals, if still good gyro is connected
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRPS = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // Read odometry and add it to a queue, then add readings to an array to be processed by the
    // logger
    inputs.odometryTimestamps =
        odometryTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value)) // Map data from queue to ->
            .toArray(Rotation2d[]::new); // this array
    // Clear the queue for the next cycle
    odometryTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
