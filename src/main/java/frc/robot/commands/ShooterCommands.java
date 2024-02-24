// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooterrefactored.Shooter;

/** Class to hold all of the commands for the Intake */
public class ShooterCommands {
  private static Command currentCommand = null;

  private static double indexerVelocityPercent = 0.0;
  private static Rotation2d anglerPosition = null;
  private static double laucnherVelocityMPS = 0.0;

  private ShooterCommands() {}

  /** Run the indexer */
  public static Command runIndexer(Shooter robotShooter, double indexerVelocityPercentSetpoint) {
    indexerVelocityPercent = indexerVelocityPercentSetpoint;

    cancelCurrentCommand();
    currentCommand =
        Commands.run(
            () ->
                robotShooter.setAllMotors(
                    indexerVelocityPercent * 12.0, anglerPosition, laucnherVelocityMPS),
            robotShooter);

    return currentCommand;
  }

  /** Run the angler */
  public static Command runAngler(Shooter robotShooter, Rotation2d anglerSetpoint) {
    anglerPosition = anglerSetpoint;

    cancelCurrentCommand();
    currentCommand =
        Commands.run(
            () ->
                robotShooter.setAllMotors(
                    indexerVelocityPercent * 12.0, anglerPosition, laucnherVelocityMPS),
            robotShooter);

    return currentCommand;
  }

  /** Run the launcher */
  public static Command runLauncher(Shooter robotShooter, double launcherVelocityMPSSetpoint) {
    laucnherVelocityMPS = launcherVelocityMPSSetpoint;

    cancelCurrentCommand();
    currentCommand =
        Commands.run(
            () ->
                robotShooter.setAllMotors(
                    indexerVelocityPercent * 12.0, anglerPosition, laucnherVelocityMPS),
            robotShooter);

    return currentCommand;
  }

  /** Stop the shooter */
  public static Command stopShooter(Shooter robotsShooter) {
    indexerVelocityPercent = 0.0;
    anglerPosition = null;
    laucnherVelocityMPS = 0.0;

    cancelCurrentCommand();
    currentCommand = Commands.run(() -> robotsShooter.stopMotors(true, true, true), robotsShooter);

    return currentCommand;
  }

  private static void cancelCurrentCommand() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }
}
