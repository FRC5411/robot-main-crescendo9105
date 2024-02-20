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
    public double leftAngleRadians = 0.0;
    public double leftVelocityRPS = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {0.0};
    public double[] leftTemperatureCelsius = new double[] {0.0};

    public double rightAngleRadians = 0.0;
    public double rightVelocityRPS = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {0.0};
    public double[] rightTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the volts for the left motor */
  public default void setLeftVolts(double volts) {}

  /** Set the volts for the right motor */
  public default void setRightVolts(double volts) {}
}
