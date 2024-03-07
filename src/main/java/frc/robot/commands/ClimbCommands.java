// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to hold of the commands for the climb */
public class ClimbCommands {
  private static Command currentCommand = null;

  private static LoggedTunableNumber leftAngleSetter =
      new LoggedTunableNumber("Climb/LeftArm/Debugging/SetpointDegrees", 0.0);
  private static LoggedTunableNumber rightAngleSetter =
      new LoggedTunableNumber("Climb/RightArm/Debugging/SetpointDegrees", 0.0);

  private static ClimbLeftDirection leftDirection = ClimbLeftDirection.STOP;
  private static ClimbRightDirection rightDirection = ClimbRightDirection.STOP;

  private static Rotation2d leftSetpoint = null;
  private static Rotation2d rightSetpoint = null;

  private ClimbCommands() {}

  /** Returns a command to run the left climb arm manually */
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

  /** Returns a command to run the right climb arm manually */
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

  /** Returns a command to run the climb manually */
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

  /** Returns a command to run the climb arms to a given setpoint */
  public static Command runClimb(Climb robotClimb) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  leftSetpoint = Rotation2d.fromDegrees(leftAngleSetter.get());
                  rightSetpoint = Rotation2d.fromDegrees(rightAngleSetter.get());
                },
                robotClimb)
            .andThen(
                Commands.runOnce(
                    () -> robotClimb.setAngle(leftSetpoint, rightSetpoint), robotClimb));

    return currentCommand;
  }

  /** Returns a command to stop the climb */
  public static Command stopClimb(Climb robotClimb) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  leftSetpoint = null;
                  rightSetpoint = null;

                  leftDirection = ClimbLeftDirection.STOP;
                  rightDirection = ClimbRightDirection.STOP;
                },
                robotClimb)
            .andThen(Commands.run(() -> robotClimb.stopMotors(), robotClimb));

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
