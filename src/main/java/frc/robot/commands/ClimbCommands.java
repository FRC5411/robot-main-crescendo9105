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

  private static ClimbLeftDirection leftDirection = ClimbLeftDirection.STOP;
  private static ClimbRightDirection rightDirection = ClimbRightDirection.STOP;

  private ClimbCommands() {}

  public static Command runLeftClimbManual(
      Climb robotClimb, ClimbLeftDirection leftDesiredDirection) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  leftDirection = leftDesiredDirection;
                },
                robotClimb)
            .andThen(
                Commands.run(() -> robotClimb.setVoltsLeft(leftDirection.getVolts()), robotClimb));

    return currentCommand;
  }

  public static Command runRightClimbManual(
      Climb robotClimb, ClimbRightDirection rightDesiredDirection) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  rightDirection = rightDesiredDirection;
                },
                robotClimb)
            .andThen(
                Commands.run(
                    () -> robotClimb.setVoltsRight(rightDirection.getVolts()), robotClimb));

    return currentCommand;
  }

  public static Command runClimbManual(
      Climb robotClimb,
      ClimbLeftDirection leftDesirecdDirection,
      ClimbRightDirection rightDesiredDirection) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  leftDirection = leftDesirecdDirection;
                  rightDirection = rightDesiredDirection;
                },
                robotClimb)
            .andThen(
                Commands.run(
                    () -> robotClimb.setVolts(leftDirection.getVolts(), rightDirection.getVolts()),
                    robotClimb));

    return currentCommand;
  }

  /** Direction of the left climb */
  public static enum ClimbLeftDirection {
    /** Run the Left Climb arm in to the robot */
    IN(12.0),
    /** Run the Left Climb arm in to the robot */
    OUT(-12.0),
    /** Stop the Left Climb wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Left Climb to spin */
    ClimbLeftDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }

  /** Direction of the right climb */
  public static enum ClimbRightDirection {
    /** Run the Right Climb arm in to the robot */
    IN(12.0),
    /** Run the Right Climb arm in to the robot */
    OUT(-12.0),
    /** Stop the Right Climb wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Right Climb to spin */
    ClimbRightDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }
}
