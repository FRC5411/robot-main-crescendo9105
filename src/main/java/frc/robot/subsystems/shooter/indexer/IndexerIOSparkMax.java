// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;

/** Class to interact with the physical indexer */
public class IndexerIOSparkMax implements IndexerIO {
  // TODO Update args as needed
  private CANSparkMax indexerMotor = new CANSparkMax(42, MotorType.kBrushless);
  private RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the indexer */
  public IndexerIOSparkMax() {
    // TODO Update as needed
    indexerMotor.clearFaults();
    indexerMotor.restoreFactoryDefaults();

    indexerMotor.setSmartCurrentLimit(40);
    indexerMotor.enableVoltageCompensation(12.0);
    indexerMotor.setIdleMode(IdleMode.kBrake);

    indexerEncoder.setPosition(0.0);

    indexerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO Convert units as needed
    inputs.angleRadians = indexerEncoder.getPosition();
    inputs.velocityRPM = indexerEncoder.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {indexerMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    indexerMotor.setVoltage(appliedVolts);
  }
}
