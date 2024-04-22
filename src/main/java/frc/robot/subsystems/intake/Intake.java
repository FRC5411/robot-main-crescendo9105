// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates.IntakeStates;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static enum IntakeSetpoint {
    IN(12.0),
    OUT(-12.0),
    STOP(6.0),
    AMP(7.0),
    OFF(0.0);

    private double volts;

    IntakeSetpoint(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return this.volts;
    }
  }

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

  public HashMap<IntakeStates, Command> mapToCommand() {
    HashMap<IntakeStates, Command> commandMap = new HashMap<>();
    commandMap.put(IntakeStates.INTAKE, runIntake(IntakeSetpoint.IN));
    commandMap.put(IntakeStates.OUTTAKE, runIntake(IntakeSetpoint.OUT));
    commandMap.put(IntakeStates.OFF, stopIntake());
    commandMap.put(IntakeStates.AMP, runIntake(IntakeSetpoint.AMP));
    return commandMap;
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
