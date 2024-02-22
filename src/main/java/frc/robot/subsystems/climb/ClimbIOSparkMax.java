// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Class to interact with the physical climb structure */
public class ClimbIOSparkMax implements ClimbIO {
  private final double GEARING = 625.0 / 1.0;

  // TODO Adjust values as needed
  private CANSparkMax leftMotor = new CANSparkMax(61, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(62, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  /** Create a new hardware implementation of the climb */
  public ClimbIOSparkMax() {
    leftMotor.clearFaults();
    rightMotor.clearFaults();
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setSmartCurrentLimit(40);
    leftMotor.enableVoltageCompensation(12.0);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(40);
    rightMotor.enableVoltageCompensation(12.0);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // TODO Add conversion factors as needed
    inputs.leftAngle = Rotation2d.fromRadians(leftEncoder.getPosition());
    inputs.leftVelocityRPS = leftEncoder.getVelocity();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.leftTemperatureCelsius = new double[] {leftMotor.getMotorTemperature()};

    inputs.rightAngle = Rotation2d.fromRadians(rightEncoder.getPosition());
    inputs.rightVelocityRPS = rightEncoder.getVelocity();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightMotor.getOutputCurrent()};
    inputs.rightTemperatureCelsius = new double[] {rightMotor.getMotorTemperature()};
  }

  @Override
  public void setLeftVolts(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    leftMotor.set(leftAppliedVolts);
  }

  @Override
  public void setRightVolts(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    rightMotor.set(rightAppliedVolts);
  }
}
