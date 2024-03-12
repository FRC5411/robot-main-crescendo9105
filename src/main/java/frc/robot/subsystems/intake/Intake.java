// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates.IntakeStates;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Direction of the Intake */
  public static enum IntakeSetpoint {
    IN(12.0),
    OUT(-12.0),
    OFF(0.0);

    private double volts;

    IntakeSetpoint(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return this.volts;
    }
  }

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/CurrentSetpoint")
  private IntakeSetpoint currentSetpoint = IntakeSetpoint.OFF;

  public Intake(IntakeIO IntakeIO) {
    this.intakeIO = IntakeIO;
  }

  public Command mapToCommand(IntakeStates desiredState) {
    switch (desiredState) {
      case INTAKE:
        return runIntake(IntakeSetpoint.IN);
      case OUTTAKE:
        return runIntake(IntakeSetpoint.OUT);
      case OFF:
      default:
        return runIntake(IntakeSetpoint.OFF);
    }
  }

  public Command runIntake(IntakeSetpoint setpoint) {
    return Commands.runOnce(() -> setCurrentSetpoint(setpoint), this);
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> setCurrentSetpoint(IntakeSetpoint.OFF), this);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
    Logger.processInputs("Intake", intakeIOInputs);

    if (currentSetpoint != null) {
      setVolts(currentSetpoint.getVolts());
    }

    if (DriverStation.isDisabled()) {
      setVolts(0);
    }
  }

  private void setCurrentSetpoint(IntakeSetpoint setpoint) {
    currentSetpoint = setpoint;
  }

  // Nulls current setpoint for manual control
  public void setVolts(double volts) {
    intakeIO.setVolts(volts);
  }

  // Nulls current setpoint for manual control
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
