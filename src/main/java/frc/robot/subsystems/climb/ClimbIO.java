// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Interface for representing the hardware */
public interface ClimbIO {
  /** Climb subsystem sensor data, positions relative to facing front of drive */
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPositionRadians = 0.0;
    public double leftVelocityRPS = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {0.0};
    public double[] leftTemperatureCelsius = new double[] {0.0};

    public double rightPositionRadians = 0.0;
    public double rightVelocityRPS = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {0.0};
    public double[] rightTemperatureCelsius = new double[] {0.0};
  }
  /** Update the inputs from the sensors */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the volts for the climb motors */
  public default void setVolts(double leftVolts, double rightVolts) {}
}
