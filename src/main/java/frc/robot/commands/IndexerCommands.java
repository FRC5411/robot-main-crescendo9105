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

  private static double indexerVolts = 0.0;

  private IndexerCommands() {}

  /** Returns a command to run the indexer motor with a given direction */
  public static Command runIndexer(Indexer robotIndexer, IndexerDirection direction) {
    setIndexerVolts(direction);

    currentCommand = Commands.run(() -> robotIndexer.setIndexerVolts(indexerVolts), robotIndexer);

    return currentCommand;
  }

  /** Returns a command to stop the Indexer motor */
  public static Command stopIndexer(Indexer robotIndexer) {
    setIndexerVolts(IndexerDirection.STOP);

    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.run(() -> robotIndexer.setIndexerVolts(indexerVolts), robotIndexer);

    return currentCommand;
  }

  /** Set the state of the indexer direciton */
  private static void setIndexerVolts(IndexerDirection direction) {
    switch (direction) {
      case IN:
        indexerVolts = -12.0;
        break;
      case OUT:
        indexerVolts = 12.0;
        break;
      case STOP:
        indexerVolts = 0.0;
        break;
      default:
        indexerVolts = 0.0;
        break;
    }

    System.out.println(
        "\nIndexerVolts: " + indexerVolts + "\nIndexerDirection: " + direction + "\n");
  }

  /** Direction of the indexer */
  public static enum IndexerDirection {
    /** Run the Indexer wheels in to the Shooter */
    IN,
    /** Run the Indexer wheels out of the Shooter */
    OUT,
    /** Stop the Indexer wheels */
    STOP
  }
}
