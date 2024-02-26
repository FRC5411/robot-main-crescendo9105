// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

/** Class to hold all of the commands for the Indexer */
public class IndexerCommands {
  private static Command currentCommand = null;

  private IndexerCommands() {}

  /** Returns a command to run the indexer motor with a given direction */
  public static Command runIndexer(Indexer robotIndexer, IndexerDirection direction) {
    logDirection(direction);

    currentCommand =
        Commands.run(() -> robotIndexer.setIndexerVolts(direction.getVolts()), robotIndexer);

    return currentCommand;
  }

  /** Returns a command to stop the Indexer motor */
  public static Command stopIndexer(Indexer robotIndexer) {
    IndexerDirection direction = IndexerDirection.STOP;
    logDirection(direction);

    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand =
        Commands.run(() -> robotIndexer.setIndexerVolts(direction.getVolts()), robotIndexer);

    return currentCommand;
  }

  /** Write the direction of the Indexer to console */
  private static void logDirection(IndexerDirection direction) {
    System.out.println(direction);
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
