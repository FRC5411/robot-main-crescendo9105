// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Interface for representing the hardware */
public interface IntakeIO {
  /** Intake subsystem sensor data */
  @AutoLog
  public static class IntakeIOInputs {
    public double angleRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the volts for the intake motor */
  public default void setVolts(double volts) {}
}
