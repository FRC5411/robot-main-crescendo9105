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

/** A Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private Pigeon2 gyro = new Pigeon2(10);

  private StatusSignal<Double> yaw = gyro.getYaw();
  private StatusSignal<Double> yawVelocity = gyro.getAngularVelocityZWorld();

  // TODO Add odometry queue

  public GyroIOPigeon2() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(250.0);
    yawVelocity.setUpdateFrequency(100.0);

    // TODO Add odometry queue
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Refresh telemetry signals, if still good gyro is connected
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRPS = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // TODO Update threaded odometry
  }
}
