// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for swerve modules */
public interface ModuleIO {
  /** Drive subsystem module sensor data */
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionM = 0.0;
    public double driveVelocityMPS = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTemperatureCelsius = new double[] {};

    public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    public Rotation2d azimuthPosition = new Rotation2d();
    public double azimuthVelocityRPS = 0.0;
    public double azimuthAppliedVolts = 0.0;
    public double[] azimuthCurrentAmps = new double[] {};
    public double[] azimuthTemperatureCelsius = new double[] {};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Set the voltage of the drive motor */
  public default void setDriveVolts(double volts) {}

  /** Set the voltage of the azimuth motor */
  public default void setAzimuthVolts(double volts) {}

  /** Set the drive motor velocity setpoint for closed-loop control */
  public default void setDriveVelocity(double velocityMPS) {}

  /** Set the azimuth motor position setpoint for closed-loop control */
  public default void setAzimuthPosition(Rotation2d position) {}

  /** Brake the drive motor */
  public default void setDriveBrake(boolean isBrake) {}

  /** Brake the azimuth motor */
  public default void setAzimuthBrake(boolean isBrake) {}

  public default void reset() {}
}
