// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import org.littletonrobotics.junction.Logger;

/** Class to interact with the physical angler structure */
public class AnglerIOSparkMax implements AnglerIO {
  private CANSparkMax anglerMotor = new CANSparkMax(41, MotorType.kBrushless);
  private Encoder rotationalEncoder = new Encoder(5, 6);
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

  private LinearFilter anglerFilter = LinearFilter.movingAverage(20);
  private double previousTimestamp = 0.0;

  private double angleOffsetRotations = 0.0;

  private double absoluteData = 0.0;
  private double relativeData = 0.0;

  private double appliedVolts = 0.0;

  /** Create a new hardware implementation of the angler */
  public AnglerIOSparkMax() {
    anglerMotor.clearFaults();
    anglerMotor.restoreFactoryDefaults();

    anglerMotor.setSmartCurrentLimit(20);
    anglerMotor.enableVoltageCompensation(12.0);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerMotor.setInverted(true);

    anglerMotor.burnFlash();

    absoluteEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0);

    angleOffsetRotations = absoluteEncoder.get();
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    // TODO Fix velocity to accurately reflect encoder readings
    inputs.anglerRelativePosition =
        Rotation2d.fromDegrees(
            Rotation2d.fromDegrees(360.0)
                .minus(
                    Rotation2d.fromRotations(
                        rotationalEncoder.getDistance() / 8192.0 + angleOffsetRotations))
                .plus(Rotation2d.fromDegrees(5.0))
                .getDegrees());
    inputs.anglerAbsolutePosition =
        Rotation2d.fromDegrees(
            Rotation2d.fromDegrees(360.0)
                .minus(Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition()))
                .plus(Rotation2d.fromDegrees(5.0))
                .getDegrees());
    inputs.anglerWeightedPosition = Rotation2d.fromRotations(getWeightedEncoderData());

    absoluteData = inputs.anglerAbsolutePosition.getRotations();
    relativeData = inputs.anglerRelativePosition.getRotations();

    inputs.anglerVelocityRadiansPerSecond = rotationalEncoder.getRate();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {anglerMotor.getMotorTemperature()};

    Logger.recordOutput("MY/FATHER", absoluteEncoder.getFrequency());
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    anglerMotor.setVoltage(appliedVolts);
  }

  /** newton raphson bompada nilfooroshan demeterio filter */
  private double getWeightedEncoderData() {
    double relative = relativeData * (7.0 / 8.0);
    double absolute = absoluteData * (1.0 / 8.0);

    double combinedData = (relativeData + absoluteData) / 2.0;

    return combinedData;
  }
}
