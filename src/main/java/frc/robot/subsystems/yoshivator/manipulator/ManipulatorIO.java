// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public Rotation2d pivotRelativePosition = new Rotation2d();
    public double pivotVelocityRadiansPerSecond = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotInternalVolts = 0.0;
    public double[] pivotAppliedCurrentAmps = new double[] {0.0};
    public double[] pivotTemperatureCelsius = new double[] {0.0};

    public double rollerVelocityRPM = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerInternalVolts = 0.0;
    public double[] rollerAppliedCurrentAmps = new double[] {0.0};
    public double[] rollerTemperatureCelsius = new double[] {0.0};
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setPivotVolts(double volts) {}

  public default void setRollerVolts(double volts) {}
}