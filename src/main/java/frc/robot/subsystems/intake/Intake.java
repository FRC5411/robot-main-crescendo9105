// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.managers.RobotSetpoints.IntakeSetpoint;;

public class Intake extends SubsystemBase {
  @AutoLogOutput(key = "Intake/CurrentSetpoint")
  private IntakeSetpoint currentSetpoint = IntakeSetpoint.OFF;

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO IntakeIO) {
    this.intakeIO = IntakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
    Logger.processInputs("Intake", intakeIOInputs);

    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    }

    if (currentSetpoint != null) {
      setVolts(currentSetpoint.getVolts());
    }
  }

  public Command runIntake(IntakeSetpoint setpoint) {
    return Commands.runOnce(() -> setCurrentSetpoint(setpoint), this);
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> setCurrentSetpoint(IntakeSetpoint.OFF), this);
  }

  public void setCurrentSetpoint(IntakeSetpoint setpoint) {
    currentSetpoint = setpoint;
  }

  public void setVolts(double volts) {
    intakeIO.setVolts(volts);
  }

  public void stopMotor() {
    intakeIO.setVolts(0.0);
  }

  // Nulls current setpoint for manual control
  public void setManualVolts(double volts) {
    setCurrentSetpoint(null);
    intakeIO.setVolts(volts);
  }

  // Nulls current setpoint for manual control
  public void stopManualMotor() {
    setCurrentSetpoint(null);
    intakeIO.setVolts(0.0);
  }
}
