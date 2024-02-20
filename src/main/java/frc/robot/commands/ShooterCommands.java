// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

/** Class to hold all of the commands for the Intake */
public class ShooterCommands {
  private static Command currentCommand = null;

  private ShooterCommands() {}

  /** Returns a command that will set the flywheels to a desired velocity */
  public static Command runFlywheels(Shooter robotShooter, double velocityMPS) {
    currentCommand =
        Commands.run(
            () -> {
              // robotShooter.setLauncherVelocity(velocityMPS);
            },
            robotShooter);

    return currentCommand;
  }

  public static Command runAll(
      Shooter robotShooter, double launcherVolts, double indexerVolts, double anglerVolts) {
    currentCommand =
        Commands.run(
            () -> {
              // robotShooter.setManualLauncher(launcherVolts);
              // robotShooter.setManualIndexer(indexerVolts);
              // robotShooter.setManualAngler(anglerVolts);
            },
            robotShooter);

    return currentCommand;
  }

  /** Stop all shooter motors */
  public static Command stopShooter(Shooter robotShooter) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = null;
        // Commands.runOnce(() -> robotShooter.stopMotors(true, true, true), robotShooter);

    return currentCommand;
  }
}
