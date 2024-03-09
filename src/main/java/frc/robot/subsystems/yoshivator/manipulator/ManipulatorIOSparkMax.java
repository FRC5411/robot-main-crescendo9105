// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Class to interact with the physical manipulator structure */
public class ManipulatorIOSparkMax implements ManipulatorIO {
  private final double PIVOT_GEARING = (64.0 / 1.0) * (3.0 / 1.0);
  private final double FLYWHEEL_GEARING = 5.0 / 1.0;

  // TODO Update as needed
  private CANSparkMax pivotMotor = new CANSparkMax(52, MotorType.kBrushless);
  private RelativeEncoder pivotRelativeEncoder = pivotMotor.getEncoder();
  private DutyCycleEncoder pivotAbsoluteEncoder = new DutyCycleEncoder(8);

  private CANSparkMax flywheelMotor = new CANSparkMax(53, MotorType.kBrushless);
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  private Rotation2d pivotAbsoluteOffset = Rotation2d.fromDegrees(114.0);

  private double pivotAppliedVolts = 0.0;
  private double flywheelAppliedVolts = 0.0;

  // Ground setpoint : -30.4
  // Idle setpoint : 100.0

  /** Create a new hardware implementation of the manipulator */
  public ManipulatorIOSparkMax() {
    pivotMotor.clearFaults();
    pivotMotor.restoreFactoryDefaults();

    // pivotMotor.setCANTimeout(20);
    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.enableVoltageCompensation(12.0);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setInverted(false);

    pivotMotor.burnFlash();

    pivotAbsoluteEncoder.setDutyCycleRange(1.0 / 8192.0, 8191.0 / 8192.0);

    flywheelMotor.clearFaults();
    flywheelMotor.restoreFactoryDefaults();

    // flywheelMotor.setCANTimeout(20);
    flywheelMotor.setSmartCurrentLimit(60);
    flywheelMotor.enableVoltageCompensation(12.0);
    flywheelMotor.setIdleMode(IdleMode.kBrake);

    flywheelMotor.setInverted(false);

    flywheelMotor.burnFlash();
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    // TODO Fix pivot position when DutyCycle is added
    inputs.pivotPosition =
        Rotation2d.fromRotations(pivotAbsoluteEncoder.get()).minus(pivotAbsoluteOffset);
    inputs.pivotRelativePosition =
        Rotation2d.fromRotations(pivotRelativeEncoder.getPosition() / PIVOT_GEARING);
    inputs.pivotVelocityRadiansPerSecond =
        Units.rotationsToRadians(pivotRelativeEncoder.getVelocity() / (PIVOT_GEARING * 60.0));
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotInternalVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotAppliedCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.pivotTemperatureCelsius = new double[] {pivotMotor.getMotorTemperature()};

    inputs.flywheelVelocityRPM = flywheelEncoder.getVelocity() / (FLYWHEEL_GEARING * 60.0);
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelInternalVolts = flywheelMotor.getBusVoltage() * flywheelMotor.getAppliedOutput();
    inputs.flywheelAppliedCurrentAmps = new double[] {flywheelMotor.getOutputCurrent()};
    inputs.flywheelTemperatureCelsius = new double[] {flywheelMotor.getMotorTemperature()};
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    pivotMotor.setVoltage(pivotAppliedVolts);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    flywheelMotor.setVoltage(flywheelAppliedVolts);
  }
}
