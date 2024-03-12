// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {
  private final double GEARING = 5.0 / 1.0;

  private CANSparkMax indexerMotor = new CANSparkMax(42, MotorType.kBrushless);
  private RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private DigitalInput beamBreakSensor = new DigitalInput(1);

  private double appliedVolts = 0.0;

  public IndexerIOSparkMax() {
    indexerMotor.clearFaults();
    indexerMotor.restoreFactoryDefaults();

    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    indexerMotor.setSmartCurrentLimit(20);
    indexerMotor.enableVoltageCompensation(12.0);
    indexerMotor.setIdleMode(IdleMode.kBrake);

    indexerMotor.setInverted(false);

    indexerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRPM = indexerEncoder.getVelocity() / GEARING;
    inputs.appliedVolts = appliedVolts;
    inputs.internalVolts = indexerMotor.getOutputCurrent() * indexerMotor.getBusVoltage();
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {indexerMotor.getMotorTemperature()};
    inputs.isBeamBroken = !beamBreakSensor.get();
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    indexerMotor.setVoltage(volts);
  }
}
