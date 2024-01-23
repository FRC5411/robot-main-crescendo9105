// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;

/** Class to interact with the physical intake structure */
public class IntakeIOSparkMax implements IntakeIO {
  // TODO Update IDs and constatns as needed
  private CANSparkMax intakeMotor = new CANSparkMax(40, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  public IntakeIOSparkMax() {
    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.enableVoltageCompensation(12.0);

    intakeEncoder.setPosition(0.0);

    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.angle = Units.Rotations.of(intakeEncoder.getPosition());
    inputs.velocity = Units.RPM.of(intakeEncoder.getVelocity());
    inputs.appliedVolts = Units.Volts.of(intakeMotor.getBusVoltage());
    inputs.appliedCurrent = Units.Amps.of(intakeMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(intakeMotor.getMotorTemperature());
  }

  @Override
  public void setVolts(Measure<Voltage> volts) {
    var adjustedVolts = MathUtil.clamp(volts.magnitude(), -12.0, 12.0);
    intakeMotor.setVoltage(adjustedVolts);
  }
}
