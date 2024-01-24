// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

/** Class to hold all of the commands for the Intake */
public class IntakeCommands {
  private static Command currentCommand = null;

  private IntakeCommands() {}

  /** Run the intake at a desired velocity in RPM */
  public static Command runIntake(Intake robotIntake, double velocityRPM) {
    currentCommand = Commands.run(() -> robotIntake.setVelocity(velocityRPM), robotIntake);

    return currentCommand;
  }

  /**
   * Returns command that sets the desired velocity to 0, will cancel any other currently running
   * Intake commands
   */
  public static Command stopIntake(Intake roboIntake) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.run(() -> roboIntake.setVelocity(0.0), roboIntake);

    return currentCommand;
  }
}
