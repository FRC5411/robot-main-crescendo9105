// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the Yoshi */
public interface YoshivatorIO {
  /** Yoshivator subsystem sensor data */
  @AutoLog
  public static class YoshivatorIOInputs {
    public Rotation2d pivotAbsolutePosition = new Rotation2d();
    public double pivotVelocityRadiansPerSecond = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotInternalVolts = 0.0;
    public double[] pivotAppliedCurrentAmps = new double[] {0.0};
    public double[] pivotTemperatureCelsius = new double[] {0.0};

    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeInternalVolts = 0.0;
    public double[] intakeAppliedCurrentAmps = new double[] {0.0};
    public double[] intakeTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(YoshivatorIOInputs inputs) {}

  /** Set the voltage of the pivot motor */
  public default void setPivotVolts(double volts) {}

  /** Set the voltage of the intake motor */
  public default void setIntakeVolts(double volts) {}
}
