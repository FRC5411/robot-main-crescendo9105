// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** A simulated swerve module */
public class ModuleIOSim implements ModuleIO {
  private final double LOOP_PERIOD_S = 0.02;

  private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim azimuthMotor = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final Rotation2d INITIAL_ABSOLUTE_ANGLE = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  public ModuleIOSim() {}

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.update(LOOP_PERIOD_S);
    azimuthMotor.update(LOOP_PERIOD_S);

    // TODO Add 250hz odometry
  }

  @Override
  public void setDriveVolts(double volts) {}

  @Override
  public void setAzimuthVolts(double volts) {}

  @Override
  public void setVelocity(double velocityMPS) {}

  @Override
  public void setAngle(double angleR) {}
}
