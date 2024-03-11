// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.Indexer;

/** Class to hold all of the commands for the Indexer */
public class IndexerCommands {
  private static Command currentCommand = null;

  private IndexerCommands() {}

  /** Returns a command to run the indexer motor with a given direction */
  public static Command runIndexer(Indexer robotIndexer, IndexerDirection direction) {

    currentCommand =
        Commands.runOnce(() -> robotIndexer.setIndexerVolts(direction.getVolts()), robotIndexer)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to stop the Indexer motor */
  public static Command stopIndexer(Indexer robotIndexer) {
    IndexerDirection direction = IndexerDirection.STOP;

    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand =
        Commands.runOnce(() -> robotIndexer.stopMotors(), robotIndexer)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to run the Indexer until a piece is stowed */
  public static Command stowPiece(Indexer robotIndexer) {
    currentCommand =
        new FunctionalCommand(
            () -> {},
            () -> {
              if (!robotIndexer.isBeamBroken()) {
                robotIndexer.setIndexerVolts(IndexerDirection.IN.getVolts() / 3.0);
              } else {
                robotIndexer.stopMotors();
              }
            },
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
}
