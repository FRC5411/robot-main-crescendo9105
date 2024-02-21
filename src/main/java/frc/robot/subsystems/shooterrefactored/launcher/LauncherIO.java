// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterrefactored.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the launcher */
public interface LauncherIO {
  /** Shooter subsystem launcher sensor data */
  @AutoLog
  public static class LauncherIOInputs {
    public Rotation2d topFlywheelPosition = new Rotation2d();
    public double topFlywheelVelocityRPM = 0.0;
    public double topFlywheelappliedVolts = 0.0;
    public double[] topFlywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] topFlywheelTemperatureCelsius = new double[] {0.0};

    public Rotation2d bottomFlywheelPosition = new Rotation2d();
    public double bottomFlywheelVelocityRPM = 0.0;
    public double bottomFlywheelAppliedVolts = 0.0;
    public double[] bottomFlywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] bottomFlywheelTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(LauncherIOInputs inputs) {}

  /** Set the voltage of the launcher motors */
  public default void setVolts(double topVolts, double bottomVolts) {}
}
