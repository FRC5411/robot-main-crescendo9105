// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemStates.IndexerStates;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  public static enum IndexerSetpoint {
    IN(12.0),
    OUT(-12.0),
    OFF(0.0),
    STOW(2.0),
    AMP(7.0);

    private double volts;

    IndexerSetpoint(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return this.volts;
    }
  }

  @AutoLogOutput(key = "Indexer/CurrentSetpoint")
  private IndexerSetpoint currentSetpoint = IndexerSetpoint.OFF;

  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Indexer", indexerIOInputs);

    if (DriverStation.isDisabled()) {
      setCurrentSetpoint(IndexerSetpoint.OFF);
      setVolts(0.0);
    }

    if (currentSetpoint != null) {
      setVolts(currentSetpoint.getVolts());
    }
  }

  public HashMap<IndexerStates, Command> mapToCommand() {
    HashMap<IndexerStates, Command> commandMap = new HashMap<>();
    commandMap.put(IndexerStates.INDEX, runIndexer(IndexerSetpoint.IN));
    commandMap.put(IndexerStates.OUTDEX, runIndexer(IndexerSetpoint.OUT));
    commandMap.put(IndexerStates.STOW, stowPiece());
    commandMap.put(IndexerStates.OFF, stopIndexer());
    commandMap.put(IndexerStates.AMP, runIndexer(IndexerSetpoint.AMP));
    return commandMap;
  }

  public Command runIndexer(IndexerSetpoint setpoint) {
    return Commands.runOnce(() -> setCurrentSetpoint(setpoint), this);
  }

  public Command stopIndexer() {
    return Commands.runOnce(() -> setCurrentSetpoint(IndexerSetpoint.OFF), this);
  }

  public Command stowPiece() {
    return new FunctionalCommand(
        () -> setCurrentSetpoint(IndexerSetpoint.STOW),
        () -> {},
        (interrupted) -> {
          setCurrentSetpoint(IndexerSetpoint.OFF);
        },
        () -> isBeamBroken(),
        this);
  }

  public void setCurrentSetpoint(IndexerSetpoint setpoint) {
    currentSetpoint = setpoint;
  }

  public boolean isBeamBroken() {
    return indexerIOInputs.isBeamBroken;
  }

  // Nulls current setpoints for manual control
  public void setVolts(double volts) {
    setCurrentSetpoint(null);
    indexerIO.setVolts(volts);
  }

  // Nulls current setpoint for manual control
  public void stopMotor() {
    setCurrentSetpoint(null);
    indexerIO.setVolts(0.0);
  }

  // Nulls current setpoints for manual control
  public void setManualVolts(double volts) {
    setCurrentSetpoint(null);
    indexerIO.setVolts(volts);
  }

  // Nulls current setpoint for manual control
  public void stopManualMotor() {
    setCurrentSetpoint(null);
    indexerIO.setVolts(0.0);
  }
}
