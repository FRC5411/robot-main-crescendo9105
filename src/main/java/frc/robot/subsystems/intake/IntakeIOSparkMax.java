// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;

/** Class to interact with the physical intake structure */
public class IntakeIOSparkMax implements IntakeIO {
  // private final double GEARING = 9.0 / 1.0;

  // TODO Update IDs and constatns as needed
  private CANSparkMax intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the intake */
  public IntakeIOSparkMax() {
    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeEncoder.setPosition(0.0);

    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.angleRotations = Rotation2d.fromRotations(intakeEncoder.getPosition());
    inputs.velocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {intakeMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {intakeMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    Logger.recordOutput("Commands running internal level", appliedVolts);
    intakeMotor.setVoltage(appliedVolts);
  }
}
