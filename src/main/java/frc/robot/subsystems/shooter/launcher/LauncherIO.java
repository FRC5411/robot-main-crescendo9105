// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the launcher */
public interface LauncherIO {
  /** Shooter subsystem launcher sensor data */
  @AutoLog
  public static class LauncherIOInputs {
    public double topFlywheelVelocityMPS = 0.0;
    public double topFlywheelAppliedVolts = 0.0;
    public double topFlywheelInternalVolts = 0.0;
    public double[] topFlywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] topFlywheelTemperatureCelsius = new double[] {0.0};
    public double topFlywheelSetpointMPS = 0.0;
    public double topFlywheelErrorMPS = 0.0;

    public double bottomFlywheelVelocityMPS = 0.0;
    public double bottomFlywheelAppliedVolts = 0.0;
    public double bottomFlywheelInternalVolts = 0.0;
    public double[] bottomFlywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] bottomFlywheelTemperatureCelsius = new double[] {0.0};
    public double bottomFlywheelSetpointMPS = 0.0;
    public double bottomFlywheelErrorMPS = 0.0;
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(LauncherIOInputs inputs) {}

  /** Set the voltage of the top launcher motor */
  public default void setTopVolts(double volts) {}

  /** Set the voltage of the bottom launcher motor */
  public default void setBottomVolts(double volts) {}

  /** Set the top motor velocity setpoint for closed-loop control */
  public default void setTopVelocity(double velocityMPS, double accelerationMPS) {}

  /** Set the bottom motor velocity setpoint for closed-loop control */
  public default void setBottomVelocity(double velocityMPS, double accelerationMPS) {}

  public default void resetTopProfile() {}

  public default void resetBottomProfile() {}
}
