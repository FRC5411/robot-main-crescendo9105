// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

/** Swerve module wrapper */
public class Module {
  // TODO Double check wheel radius
  private static final double WHEEL_RADIUS_M = Units.inchesToMeters(3.0);
  public static final double ODOMETRY_FREQUENCY = 250.0;
  private final int MODULE_ID;

  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged moduleIOInputs = new ModuleIOInputsAutoLogged();

  private Double velocitySetpoint = null;
  private Rotation2d angleSetpoint = null;

  private Rotation2d azimuthRelativeOffset = null;
  private double lastPositionM = 0.0;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {};

  /** Creates a new swerve module */
  public Module(ModuleIO io, int id) {
    moduleIO = io;
    MODULE_ID = id;
  }

  /** Update inputs without running periodic logic; Odometry updates need to be properly thread locked */
  public void updateInputs() {
    moduleIO.updateInputs(moduleIOInputs);
  }

  /** Called in subsystem periodic */
  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(MODULE_ID), moduleIOInputs);
  }
}
