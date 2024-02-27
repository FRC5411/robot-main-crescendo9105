// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the indexer */
public interface IndexerIO {
  /** Shooter subsystem indexer sensor data */
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerVelocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};

    public boolean isBeamBroken = false;
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Set the voltage of the indexer motor */
  public default void setVolts(double volts) {}
}
