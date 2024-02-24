// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

/** Class to hold all of the commands for the Shooter */
public class ShooterCommands {
  private static Command currentCommand = null;

  private static Rotation2d anglerPosition = null;
  private static double laucnherVelocityMPS = 0.0;

  private ShooterCommands() {}

  /** Returns a command to run the angler motor */
  public static Command runAngler(Shooter robotShooter, Rotation2d anglerPositionSetpoint) {
    anglerPosition = anglerPositionSetpoint;

    robotShooter.resetAnglerFeedback();
    currentCommand =
        Commands.runOnce(
            () -> robotShooter.setAllMotors(anglerPosition, laucnherVelocityMPS), robotShooter);

    return currentCommand;
  }

  /** Returns a command to run the launcher motors */
  public static Command runLauncher(Shooter robotsShooter, double launcherVelocityMPSSetpoint) {
    laucnherVelocityMPS = launcherVelocityMPSSetpoint;
    currentCommand =
        Commands.runOnce(
            () -> robotsShooter.setAllMotors(anglerPosition, laucnherVelocityMPS), robotsShooter);

    return currentCommand;
  }

  /** Returns a command to run the angler manually */
  public static Command runAnglerManual(Shooter robotShooter, double anglerVolts) {
    robotShooter.setAnglerPosition(null);

    currentCommand = Commands.runOnce(() -> robotShooter.setAnglerVolts(anglerVolts), robotShooter);

    return currentCommand;
  }

  /** Returns a command to run the launcher manually */
  public static Command runLauncherManual(Shooter robotShooter, double launcherVolts) {
    robotShooter.setLauncherVelocityMPS(null);

    currentCommand =
        Commands.runOnce(
            () -> robotShooter.setLauncherVolts(launcherVolts, launcherVolts), robotShooter);

    return currentCommand;
  }

  /** Returns a command to stop all Shooter motors */
  public static Command stopShooter(Shooter robotsShooter) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.run(() -> robotsShooter.stopMotors(true, true), robotsShooter);

    return currentCommand;
  }
}
