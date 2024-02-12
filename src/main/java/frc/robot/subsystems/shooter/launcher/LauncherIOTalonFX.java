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
  // TODO Update as needed
  private TalonFX topMotor = new TalonFX(50);
  private TalonFX bottomMotor = new TalonFX(51);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  // TODO Add closed loop control

  /** Create a new hardware implementation of the launcher */
  public LauncherIOTalonFX() {
    // TODO Update as needed
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

    topMotor.setInverted(true);
    bottomMotor.setInverted(false);

    topMotor.getConfigurator().apply(leftConfiguration);
    bottomMotor.getConfigurator().apply(rightConfiguration);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // TODO Update as needed
    inputs.topAngleRadians = topMotor.getPosition().getValueAsDouble();
    inputs.topVelocityRPM = topMotor.getVelocity().getValueAsDouble();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getStatorCurrent().getValueAsDouble()};
    inputs.topTemperatureCelsius = new double[] {topMotor.getDeviceTemp().getValueAsDouble()};

    inputs.bottomAngleRadians = bottomMotor.getPosition().getValueAsDouble();
    inputs.bottomVelocityRPM = bottomMotor.getVelocity().getValueAsDouble();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps =
        new double[] {bottomMotor.getStatorCurrent().getValueAsDouble()};
    inputs.bottomTemperatureCelsius = new double[] {bottomMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setVolts(double leftVolts, double rightVolts) {
    topAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    topMotor.setVoltage(topAppliedVolts);
    bottomMotor.setVoltage(bottomAppliedVolts);
  }
}
