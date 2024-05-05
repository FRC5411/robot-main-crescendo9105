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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Intake extends SubsystemBase {
  public static enum IntakeSetpoint {
    IN(12.0),
    OUT(-12.0),
    STOW(12.0, 2.0),
    AMP(7.0),
    OFF(0.0);

    private double intakeVolts;
    private double indexerVolts;

    IntakeSetpoint(double intakeVolts, double indexerVolts) {
      this.intakeVolts = intakeVolts;
      this.indexerVolts = indexerVolts;
    }

    IntakeSetpoint(double volts) {
      this(volts, volts);
    }

    public double getIntakeVolts() {
      return this.intakeVolts;
    }

    public double getIndexerVolts() {
      return this.indexerVolts;
    }
  }

  @AutoLogOutput(key = "Intake/CurrentSetpoint")
  private IntakeSetpoint currentSetpoint = IntakeSetpoint.OFF;

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO, IndexerIO indexerIO) {
    this.intakeIO = intakeIO;
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
    Logger.processInputs("Intake", intakeIOInputs);

    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Index", indexerIOInputs);

    if (currentSetpoint != null) {
      setVolts(currentSetpoint.getIntakeVolts(), currentSetpoint.getIndexerVolts());
    }

    if (DriverStation.isDisabled()) {
      setVolts(0.0);
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

  public Command stowPiece() {
    return new FunctionalCommand(
        () -> setCurrentSetpoint(IntakeSetpoint.STOW),
        () -> {},
        (interrupted) -> {
          setCurrentSetpoint(IntakeSetpoint.OFF);
        },
        () -> isBeamBroken(),
        this);
  }

  public void setCurrentSetpoint(IntakeSetpoint setpoint) {
    currentSetpoint = setpoint;
  }

  public void setVolts(double intakeV, double indexV) {
    intakeIO.setVolts(intakeV);
    indexerIO.setVolts(indexV);
  }

  public void setVolts(double volts) {
    setVolts(volts, volts);
  }

  public void stopMotor() {
    intakeIO.setVolts(0.0);
    indexerIO.setVolts(0.0);
  }

  public boolean isBeamBroken() {
    return indexerIOInputs.isBeamBroken;
  }

  // Nulls current setpoint for manual control
  public void setManualVolts(double intakeV, double indexV) {
    setCurrentSetpoint(null);
    setVolts(intakeV, indexV);
  }

  // Nulls current setpoint for manual control
  public void setManualVolts(double volts) {
    setManualVolts(volts);
  }

  // Nulls current setpoint for manual control
  public void stopManualMotor() {
    setCurrentSetpoint(null);
    stopMotor();
  }
}