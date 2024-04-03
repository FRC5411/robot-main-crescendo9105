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
import frc.robot.managers.RobotSetpoints.IndexerSetpoint;;

public class Indexer extends SubsystemBase {
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

  public Command runIndexer(IndexerSetpoint setpoint) {
    return Commands.runOnce(() -> setCurrentSetpoint(setpoint), this);
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

  public void setVolts(double volts) {
    indexerIO.setVolts(volts);
  }

  public void stopMotor() {
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
