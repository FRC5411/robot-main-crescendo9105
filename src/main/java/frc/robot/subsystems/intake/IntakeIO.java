// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Interface for representing the hardware */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Measure<Angle> angle = null;
    public Measure<Velocity<Angle>> velocity = null;
    public Measure<Voltage> appliedVolts = null;
    public Measure<Current> appliedCurrent = null;
    public Measure<Temperature> temperature = null;
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the volts of the intake motor */
  public default void setVolts(Measure<Voltage> volts) {}
}
