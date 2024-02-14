// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

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

  /** Stop the current Climb command */
  public static void cancelCurrentCommand() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }
}
