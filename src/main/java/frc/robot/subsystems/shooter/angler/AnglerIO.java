// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the angler */
public interface AnglerIO {
  /** Shooter subsystem angler sensor data */
  @AutoLog
  public static class AnglerIOInputs {
    public Rotation2d anglerAbsolutePosition = new Rotation2d();
    public Rotation2d anglerRelativePosition = new Rotation2d();
    public int anglerDutyCycleFrequency = 0;
    public double anglerVelocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double internalVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(AnglerIOInputs inputs) {}

  /** Set the voltage of the angler motor */
  public default void setVolts(double volts) {}
}
