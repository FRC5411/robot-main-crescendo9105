// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;

/** Class to interact with the physical angler */
public class AnglerIOSparkMax implements AnglerIO {
  // TODO Update args as needed
  private CANSparkMax anglerMotor = new CANSparkMax(41, MotorType.kBrushless);
  private RelativeEncoder anglerEncoder = anglerMotor.getEncoder();

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the angler */
  public AnglerIOSparkMax() {
    anglerMotor.clearFaults();
    anglerMotor.restoreFactoryDefaults();

    anglerMotor.setSmartCurrentLimit(40);
    anglerMotor.enableVoltageCompensation(12.0);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerEncoder.setPosition(0.0);

    anglerMotor.burnFlash();
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    // TODO Convert units as needed
    inputs.angleRadians = anglerEncoder.getPosition();
    inputs.velocityRPS = anglerEncoder.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {anglerMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    anglerMotor.setVoltage(appliedVolts);
  }
}
