// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for swerve modules */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionM = 0.0;
    public double driveVelocityMPS = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    public Rotation2d azimuthPosition = new Rotation2d();
    public double azimuthVelocityRPS = 0.0;
    public double azimuthAppliedVolts = 0.0;
    public double[] azimuthCurrentAmps = new double[] {};

    public double[] odometryDrivePositionR = new double[] {};
    public Rotation2d[] odometryAzimuthPositions = new Rotation2d[] {};
  }

  /** Update the hardware inputs for a module */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Set the voltage for the drive motor */
  public default void setDriveVolts(double volts) {}

  /** Set the voltage for the azmith motor */
  public default void setAzimuthVolts(double volts) {}

  /** Enable closed loop control for the module's drive velocity */
  public default void setVelocity(double velocityMPS) {}

  /** Enable closed loop control for the module's azimuth position */
  public default void setAngle(double angleR) {}

  /** Brake the drive motor */
  public default void setDriveBrake(boolean isBrake) {}

  /** Brake the azimuth motor */
  public default void setAzimuthBrake(boolean isBrake) {}
}
