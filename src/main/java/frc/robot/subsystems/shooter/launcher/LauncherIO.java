// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.AutoLog;

/** Interface for representing the hardware */
public interface LauncherIO {
  /** Launcher sensor data */
  @AutoLog
  public static class LauncherIOInputsI {
    public double topAngleRadians = 0.0;
    public double topVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topAppliedCurrentAmps = new double[] {0.0};
    public double[] topTemperatureCelsius = new double[] {0.0};

    public double bottomAngleRadians = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomAppliedCurrentAmps = new double[] {0.0};
    public double[] bottomTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(LauncherIOInputsI inputs) {}

  /** Set the volts for the launcher motor */
  public default void setVolts(double topVolts, double bottomVolts) {}

  /** Set the closed-loop velocity for the motors */
  public default void setVelocity(double velocityMPS) {}
}
