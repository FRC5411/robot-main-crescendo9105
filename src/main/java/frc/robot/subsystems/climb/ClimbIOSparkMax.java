// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimbIOSparkMax implements ClimbIO {
  private final double GEARING = 125.0 / 1.0;

  private CANSparkMax leftMotor = new CANSparkMax(61, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(62, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private DutyCycleEncoder leftAbsoluteEncoder = new DutyCycleEncoder(4);
  private DutyCycleEncoder rightAbsoluteEncoder = new DutyCycleEncoder(3);

  private Rotation2d leftEncoderOffset = Rotation2d.fromDegrees(-302.0);
  private Rotation2d rightEncoderOffset = Rotation2d.fromDegrees(-58.0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  public ClimbIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.clearFaults();
    rightMotor.clearFaults();

    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    leftMotor.setSmartCurrentLimit(40);
    leftMotor.enableVoltageCompensation(12.0);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(40);
    rightMotor.enableVoltageCompensation(12.0);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setInverted(true);
    rightMotor.setInverted(true);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    leftMotor.follow(rightMotor);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // TODO Add conversion factors as needed
    inputs.leftPosition =
        Rotation2d.fromRotations(leftAbsoluteEncoder.getAbsolutePosition()).plus(leftEncoderOffset);
    inputs.leftVelocityRPS =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEARING);
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.leftTemperatureCelsius = new double[] {leftMotor.getMotorTemperature()};

    inputs.rightPosition =
        Rotation2d.fromDegrees(360.0)
            .minus(Rotation2d.fromRotations(rightAbsoluteEncoder.getAbsolutePosition()))
            .plus(rightEncoderOffset);
    inputs.rightVelocityRPS =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEARING);
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
