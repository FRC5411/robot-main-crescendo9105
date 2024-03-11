// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IndexerCommands.IndexerDirection;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  private static Command currentCommand = null;

  /** Creates a new Indexer. */
  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

    /** Returns a command to run the indexer motor with a given direction */
  public static Command runIndexer(Indexer robotIndexer, IndexerDirection direction) {
    currentCommand =
        Commands.runOnce(() -> {
          robotIndexer.setIndexerVolts(direction.getVolts());
          logDirection(direction);
        }, robotIndexer);

    return currentCommand;
  }

  /** Returns a command to stop the Indexer motor */
  public static Command stopIndexer(Indexer robotIndexer) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand =
        Commands.runOnce(() -> {
          robotIndexer.stopMotors();
          logDirection(IndexerDirection.STOP);
        }, robotIndexer);

    return currentCommand;
  }

  /** Returns a command to run the Indexer until a piece is stowed */
  public static Command stowPiece(Indexer robotIndexer) {
    currentCommand =
        new FunctionalCommand(
            () -> {
              robotIndexer.setIndexerVolts(IndexerDirection.IN.getVolts());
            },
            () -> {},
            (interrupted) -> {
              robotIndexer.stopMotors();
            },
            () -> robotIndexer.isBeamBroken(),
            robotIndexer);

    return currentCommand;
  }

  /** Write the direction of the Indexer to console */
  private static void logDirection(IndexerDirection direction) {
    System.out.println("INDEXER: " + direction);
  }

  /** Direction of the indexer */
  public static enum IndexerDirection {
    /** Run the Indexer wheels in to the Shooter */
    IN(12.0),
    /** Run the Indexer wheels out of the Shooter */
    OUT(-12.0),
    /** Stop the Indexer wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Indexer to spin */
    IndexerDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }


  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Indexer/Inputs", indexerIOInputs);
  }

  /** Stop the indexer motor */
  public void stopMotors() {
    indexerIO.setVolts(0.0);
  }

  /** Set the voltage of the indexer motor */
  public void setIndexerVolts(double volts) {
    indexerIO.setVolts(volts);
  }

  /** Returns the status of the beam break sensor; checks if a note is stowed or not */
  public boolean isBeamBroken() {
    return indexerIOInputs.isBeamBroken;
  }
}
