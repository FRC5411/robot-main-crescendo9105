// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator.manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware interface for the manipulator */
public interface ManipulatorIO {
  /** Yoshivator subsystem manipulator sensor data */
  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public double pivotVelocityRadiansPerSecond = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotInternalVolts = 0.0;
    public double[] pivotAppliedCurrentAmps = new double[] {0.0};
    public double[] pivotTemperatureCelsius = new double[] {0.0};

    public double flywheelVelocityRPM = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelInternalVolts = 0.0;
    public double[] flywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] flywheelTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /** Set the voltage of the pivot motor */
  public default void setPivotVolts(double volts) {}

  /** Set the voltage of the flywheel motor */
  public default void setFlywheelVolts(double volts) {}
}
