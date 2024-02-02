// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

/** Class to interact with the physical launcher */
public class LauncherIOTalonFX implements LauncherIO {
  private TalonFX leftMotor = new TalonFX(50);
  private TalonFX rightMotor = new TalonFX(51);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  /** Create a new hardware implementation of the launcher */
  public LauncherIOTalonFX() {
    TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();

    leftConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    rightConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfiguration.CurrentLimits.StatorCurrentLimit = 40;

    leftConfiguration.Voltage.PeakForwardVoltage = 12.0;
    leftConfiguration.Voltage.PeakReverseVoltage = -12.0;
    rightConfiguration.Voltage.PeakForwardVoltage = 12.0;
    rightConfiguration.Voltage.PeakReverseVoltage = -12.0;

    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.getConfigurator().apply(leftConfiguration);
    rightMotor.getConfigurator().apply(rightConfiguration);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.leftAngleRadians = leftMotor.getPosition().getValueAsDouble();
    inputs.leftVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftAppliedCurrentAmps = new double[] {leftMotor.getStatorCurrent().getValueAsDouble()};
    inputs.leftTemperatureCelsius = new double[] {leftMotor.getDeviceTemp().getValueAsDouble()};

    inputs.rightAngleRadians = rightMotor.getPosition().getValueAsDouble();
    inputs.rightVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightAppliedCurrentAmps = new double[] {rightMotor.getStatorCurrent().getValueAsDouble()};
    inputs.rightTemperatureCelsius = new double[] {rightMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setVolts(double leftVolts, double rightVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    leftMotor.setVoltage(leftAppliedVolts);
    rightMotor.setVoltage(rightAppliedVolts);
  }
}
