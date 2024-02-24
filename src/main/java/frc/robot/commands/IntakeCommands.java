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

  private static double intakeVolts = 0.0;

  private IntakeCommands() {}

  /** Returns a command to run the Intake motor with a given direction */
  public static Command runIntake(Intake robotIntake, IntakeDirection direction) {
    setIntakeVolts(direction);

    currentCommand = Commands.run(() -> robotIntake.setVolts(intakeVolts), robotIntake);

    return currentCommand;
  }

  /** Returns a command to stop the Intake motor */
  public static Command stopIntake(Intake robotIntake) {
    setIntakeVolts(IntakeDirection.STOP);

    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.run(() -> robotIntake.setVolts(intakeVolts), robotIntake);

    return currentCommand;
  }

  /** Set the state of the indexer direciton */
  private static void setIntakeVolts(IntakeDirection direction) {
    switch (direction) {
      case IN:
        intakeVolts = -12.0;
        break;
      case OUT:
        intakeVolts = 12.0;
        break;
      case STOP:
        intakeVolts = 0.0;
        break;
      default:
        intakeVolts = 0.0;
        break;
    }

    System.out.println(
        "\nIntakeVolts: " + intakeVolts + "\nIntakeDirection: " + direction + "\n");
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
