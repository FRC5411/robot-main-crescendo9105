// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /** Direction of the indexer */
  public static enum IndexerSetpoint {
    IN(12.0),
    OUT(-12.0),
    IDLE(0.0),
    STOW(4.0);

    private double volts;

    /** Define the direction for the Indexer to spin */
    IndexerSetpoint(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }

  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  @AutoLogOutput(key = "Indexer/CurrentCommand")
  private Command currentCommand = null;

  @AutoLogOutput(key = "Indexer/CurrentSetpoint")
  private IndexerSetpoint currentSetpoint = IndexerSetpoint.IDLE;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  public Command runIndexer(IndexerSetpoint setpoint) {
    currentCommand = Commands.runOnce(() -> setCurrentSetpoint(setpoint), this);

    return currentCommand;
  }

  public Command stopIndexer() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.runOnce(() -> setCurrentSetpoint(IndexerSetpoint.IDLE), this);

    return currentCommand;
  }

  public Command stowPiece() {
    currentCommand =
        new FunctionalCommand(
            () -> setCurrentSetpoint(IndexerSetpoint.STOW),
            () -> {},
            (interrupted) -> {
              setCurrentSetpoint(IndexerSetpoint.IDLE);
            },
            () -> isBeamBroken(),
            this);

    return currentCommand;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Indexer", indexerIOInputs);

    if (currentSetpoint != null) {
      setVolts(currentSetpoint.getVolts());
    }

    if (DriverStation.isDisabled()) {
      setCurrentSetpoint(IndexerSetpoint.IDLE);
      setVolts(0);
    }
  }

  private void setCurrentSetpoint(IndexerSetpoint setpoint) {
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
