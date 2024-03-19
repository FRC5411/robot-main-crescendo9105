// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Class to represent the swerve module in simulation */
public class ModuleIOSim implements ModuleIO {
  private final double LOOP_PERIOD_S = 0.02;
  private final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;
  private final double CIRCUMFRENCE_METERS = 2 * Math.PI * (5.08 / 100);

  private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), DRIVE_GEAR_RATIO, 0.025);
  private DCMotorSim azimuthMotor = new DCMotorSim(DCMotor.getNEO(1), AZIMUTH_GEAR_RATIO, 0.004);

  private final Rotation2d INITIAL_ABSOLUTE_ANGLE = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  private PIDController driveFeedback = new PIDController(0.002, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.15, 0.0);

  private PIDController azimuthFeedback = new PIDController(2.7, 0.0, 0.0);

  /** Create a new virtual implementation of a swerve module */
  public ModuleIOSim(int module) {
    azimuthFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.update(LOOP_PERIOD_S);
    azimuthMotor.update(LOOP_PERIOD_S);

    inputs.drivePositionM = driveMotor.getAngularPositionRotations() * CIRCUMFRENCE_METERS;
    inputs.driveVelocityMPS = driveMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};
    inputs.driveTemperatureCelsius = new double[] {0.0};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(azimuthMotor.getAngularPositionRad()).plus(INITIAL_ABSOLUTE_ANGLE);
    inputs.azimuthPosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
    inputs.azimuthVelocityRPS = azimuthMotor.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};
    inputs.azimuthTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVolts(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthMotor.setInputVoltage(azimuthAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double velocityMPS) {
    var feedbackOutput =
        driveFeedback.calculate(driveMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS / 60.0);
    feedbackOutput += driveFeedforward.calculate(velocityMPS);
    setDriveVolts(feedbackOutput);
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    var feedbackOutput =
        azimuthFeedback.calculate(azimuthMotor.getAngularPositionRad(), position.getRadians());

    setAzimuthVolts(feedbackOutput);
  }
}
