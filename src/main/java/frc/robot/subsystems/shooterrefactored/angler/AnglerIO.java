// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterrefactored.angler;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware interface for the angler */
public interface AnglerIO {
  /** Shooter subsystem angler sensor data */
  @AutoLog
  public static class AnglerIOInputsR {
    public Rotation2d anglerPosition = new Rotation2d();
    public double anglerVelocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0}; 
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(AnglerIOInputsR inputs) {}

  /** Set the voltage of the intake motor */
  public default void setVolts(double volts) {}
}
