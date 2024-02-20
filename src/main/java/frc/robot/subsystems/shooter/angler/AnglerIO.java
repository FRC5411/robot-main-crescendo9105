// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import org.littletonrobotics.junction.AutoLog;

/** Interface for representing the hardware */
public interface AnglerIO {
  /** Angler sensor data */
  @AutoLog
  public static class AnglerIOInputs {
    public double angleRadians = 0.0;
    public double dutyCycleEncoderRadians = 0.0;
    public double velocityRPS = 0.0;
    public double dutyCycleEncoderRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(AnglerIOInputs inputs) {}

  /** Set the volts for the angler motor */
  public default void setVolts(double volts) {}
}
