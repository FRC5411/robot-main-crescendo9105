// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;

/** Class to interact with the physical launcher */
public class LauncherIOSparkMax implements LauncherIO {
  private CANSparkMax topMotor = new CANSparkMax(43, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(44, MotorType.kBrushless);

  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  /** Create a new hardware implementation of the launcher */
  public LauncherIOSparkMax() {
    // TODO Update as needed
    topMotor.clearFaults();
    bottomMotor.clearFaults();
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setSmartCurrentLimit(40);
    bottomMotor.setSmartCurrentLimit(40);
    topMotor.enableVoltageCompensation(12.0);
    bottomMotor.enableVoltageCompensation(12.0);
    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setIdleMode(IdleMode.kBrake);

    topEncoder.setPosition(0.0);
    bottomEncoder.setPosition(0.0);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // TODO Update as needed
    inputs.topAngleRadians = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getOutputCurrent()};
    inputs.topTemperatureCelsius = new double[] {topMotor.getMotorTemperature()};

    inputs.bottomAngleRadians = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps = new double[] {bottomMotor.getOutputCurrent()};
    inputs.bottomTemperatureCelsius = new double[] {bottomMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    topMotor.setVoltage(topAppliedVolts);
    bottomMotor.setVoltage(bottomAppliedVolts);
  }
}
