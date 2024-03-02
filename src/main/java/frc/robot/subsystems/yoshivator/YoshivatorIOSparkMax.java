// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Class to interact with the physical yoshivator structure */
public class YoshivatorIOSparkMax implements YoshivatorIO {
  private final double PIVOT_GEARING = 75.0 / 1.0;
  private final double INTAKE_GEARING = 3.0 / 1.0;
  
  // TODO Update as needed
  private CANSparkMax pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private double pivotAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  /** Create a new hardware implementation of the yoshi */
  public YoshivatorIOSparkMax() {
    pivotMotor.clearFaults();
    pivotMotor.restoreFactoryDefaults();

    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.enableVoltageCompensation(12.0);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setInverted(true);

    pivotMotor.burnFlash();

    pivotEncoder.setDutyCycleRange(1.0 / 8192.0, 8191.0 / 8192.0);

    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setInverted(true);

    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(YoshivatorIOInputs inputs) {
    // TODO Fix
    inputs.pivotAbsolutePosition = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition());
    inputs.pivotVelocityRadiansPerSecond = pivotEncoder.getDistance();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotInternalVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotAppliedCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.pivotTemperatureCelsius = new double[] {pivotMotor.getMotorTemperature()};

    // TODO Add intake stuff
  }

  @Override
  public void setPivotVolts(double volts) {}

  @Override
  public void setIntakeVolts(double volts) {}
}
