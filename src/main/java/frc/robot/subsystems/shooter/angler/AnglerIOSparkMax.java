// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Class to interact with the physical angler structure */
public class AnglerIOSparkMax implements AnglerIO {
  private CANSparkMax anglerMotor = new CANSparkMax(41, MotorType.kBrushless);
  private DutyCycleEncoder anglerEncoder = new DutyCycleEncoder(0);

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the angler */
  public AnglerIOSparkMax() {
    anglerMotor.clearFaults();
    anglerMotor.restoreFactoryDefaults();

    anglerMotor.setSmartCurrentLimit(40);
    anglerMotor.enableVoltageCompensation(12.0);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerMotor.setInverted(true);

    anglerMotor.burnFlash();
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    // TODO Fix velocity to accurately reflect encoder readings
    inputs.anglerPosition =
        Rotation2d.fromDegrees(360.0)
            .minus(Rotation2d.fromRotations((anglerEncoder.getAbsolutePosition())))
            .plus(Rotation2d.fromDegrees(5.0));
    inputs.anglerVelocityRadiansPerSecond = anglerEncoder.getDistance();
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
