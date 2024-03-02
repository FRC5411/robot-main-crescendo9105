// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import org.littletonrobotics.junction.Logger;

/** Class to hold of the commands for the climb */
public class ClimbCommands {
  private static Command currentCommand = null;

  private ClimbCommands() {}

  /** Set the angle of both climb arms in radians */
  public static Command setAngle(
      Climb robotClimb, double leftAngleRadians, double rightAngleRadians) {
    currentCommand =
        Commands.run(() -> robotClimb.setAngle(leftAngleRadians, rightAngleRadians), robotClimb);

    return currentCommand;
  }

  /** Set the manual voltage of each arm */
  public static Command runManual(
      Climb robotClimb, double leftSideVolts, double rightSideVolts, boolean invertArms) {
    if (invertArms) {
      currentCommand =
          Commands.run(() -> robotClimb.setVolts(-leftSideVolts, -rightSideVolts), robotClimb);
    } else {
      currentCommand =
          Commands.run(() -> robotClimb.setVolts(leftSideVolts, rightSideVolts), robotClimb);
    }
    Logger.recordOutput("Climb/Direction", invertArms);

    return currentCommand;
  }

  /** Set the manual voltage of each arm */
  public static Command runManualLeft(Climb robotClimb, double volts, boolean invertArms) {
    if (invertArms) {
      currentCommand = Commands.run(() -> robotClimb.setVoltsLeft(-volts), robotClimb);
    } else {
      currentCommand = Commands.run(() -> robotClimb.setVoltsLeft(volts), robotClimb);
    }
    Logger.recordOutput("Climb/Direction", invertArms);

    return currentCommand;
  }

  /** Set the manual voltage of each arm */
  public static Command runManualRight(Climb robotClimb, double volts, boolean invertArms) {
    if (invertArms) {
      currentCommand = Commands.run(() -> robotClimb.setVoltsRight(-volts), robotClimb);
    } else {
      currentCommand = Commands.run(() -> robotClimb.setVoltsRight(volts), robotClimb);
    }
    Logger.recordOutput("Climb/Direction", invertArms);

    return currentCommand;
  }

  /** Stop the climb */
  public static Command stopClimb(Climb robotClimb) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentCommand = Commands.run(() -> robotClimb.setVolts(0.0, 0.0), robotClimb);

    return currentCommand;
  }

  // /** Flips the direction of the climb motors */
  // public static void flipClimbDirection(boolean flipDirection) {
  //   invertArms = flipDirection;

  //   Logger.recordOutput("Climb/Direction", invertArms);
  // }

  /** Stop the current Climb command */
  public static void cancelCurrentCommand() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }
}
