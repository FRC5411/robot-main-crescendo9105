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
import edu.wpi.first.wpilibj.Encoder;
import org.littletonrobotics.junction.Logger;

/** Class to interact with the physical angler structure */
public class AnglerIOSparkMax implements AnglerIO {
  private CANSparkMax anglerMotor = new CANSparkMax(41, MotorType.kBrushless);

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);
  private Encoder relativeEncoder = new Encoder(5, 6);

  private Rotation2d absoluteEncoderOffset = Rotation2d.fromDegrees(179.0);

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the angler */
  public AnglerIOSparkMax() {
    anglerMotor.clearFaults();
    anglerMotor.restoreFactoryDefaults();

    anglerMotor.setSmartCurrentLimit(60);
    anglerMotor.enableVoltageCompensation(12.0);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerMotor.setInverted(true);

    anglerMotor.burnFlash();

    absoluteEncoder.setDutyCycleRange(1.0 / 8192.0, 8191.0 / 8192.0);
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    // TODO Fix velocity to accurately reflect encoder readings
    inputs.anglerAbsolutePosition =
        Rotation2d.fromDegrees(360.0)
            .minus(
                Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition())
                    .plus(absoluteEncoderOffset));
    inputs.anglerRelativePosition = Rotation2d.fromRotations(relativeEncoder.get() / 2048.0);
    inputs.anglerDutyCycleFrequency = absoluteEncoder.getFrequency();
    inputs.anglerVelocityRadiansPerSecond = relativeEncoder.getRate();
    inputs.appliedVolts = appliedVolts;
    inputs.internalVolts = anglerMotor.getBusVoltage() * anglerMotor.getAppliedOutput();
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {anglerMotor.getMotorTemperature()};

    Logger.recordOutput("Shooter/Angler/AppliedOutput", anglerMotor.getAppliedOutput());
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    anglerMotor.setVoltage(appliedVolts);
  }
}
