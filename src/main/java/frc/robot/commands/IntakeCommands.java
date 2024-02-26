// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

/** Class to hold all of the commands for the Intake */
public class IntakeCommands {
  private static Command currentCommand = null;

  private IntakeCommands() {}

  /** Returns a command to run the Intake motor with a given direction */
  public static Command runIntake(Intake robotIntake, IntakeDirection direction) {
    logDirection(direction);

    currentCommand =
        Commands.run(() -> robotIntake.setVolts(direction.getVolts()), robotIntake)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to stop the Intake motor */
  public static Command stopIntake(Intake robotIntake) {
    IntakeDirection direction = IntakeDirection.STOP;
    logDirection(direction);

    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand =
        Commands.run(() -> robotIntake.setVolts(direction.getVolts()), robotIntake)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Write the direction of the Intake to console */
  private static void logDirection(IntakeDirection direction) {
    System.out.println(direction);
  }

  /** Direction of the Intake */
  public static enum IntakeDirection {
    /** Run the Intake wheels in to the Shooter */
    IN(12.0),
    /** Run the Intake wheels out of the Shooter */
    OUT(-12.0),
    /** Stop the Intake wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Intake to spin */
    IntakeDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }
}
