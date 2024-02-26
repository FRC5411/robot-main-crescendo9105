// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/** Class to hold all of the commands for the Intake */
public class IntakeCommands {
  private static Command currentCommand = null;

  private static double intakeVolts = 0.0;

  private IntakeCommands() {}

  /** Returns a command to run the Intake motor with a given direction */
  public static Command runIntake(Intake robotIntake, IntakeDirection direction) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  setIntakeVolts(direction);
                },
                robotIntake)
            .andThen(Commands.run(() -> robotIntake.setVolts(intakeVolts), robotIntake));

    return currentCommand;
  }

  /** Returns a command to stop the Intake motor */
  public static Command stopIntake(Intake robotIntake) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand =
        Commands.runOnce(
                () -> {
                  setIntakeVolts(IntakeDirection.STOP);
                },
                robotIntake)
            .andThen(Commands.run(() -> robotIntake.setVolts(intakeVolts), robotIntake));

    return currentCommand;
  }

  /** Set the state of the indexer direciton */
  private static void setIntakeVolts(IntakeDirection direction) {
    switch (direction) {
      case IN:
        intakeVolts = -12.0;
        Logger.recordOutput("Intake/Direction", "IN");
        break;
      case OUT:
        intakeVolts = 12.0;
        Logger.recordOutput("Intake/Direction", "OUT");
        break;
      case STOP:
        intakeVolts = 0.0;
        Logger.recordOutput("Intake/Direction", "STOP");
        break;
      default:
        intakeVolts = 0.0;
        Logger.recordOutput("Intake/Direction", "STOPDEF");
        break;
    }

    System.out.println("\nIntakeVolts: " + intakeVolts + "\nIntakeDirection: " + direction + "\n");
  }

  /** Direction of the Intake */
  public static enum IntakeDirection {
    /** Run the Intake wheels in to the Shooter */
    IN,
    /** Run the Intake wheels out of the Shooter */
    OUT,
    /** Stop the Intake wheels */
    STOP
  }
}
