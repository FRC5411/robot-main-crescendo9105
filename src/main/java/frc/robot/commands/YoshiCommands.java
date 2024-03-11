// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.BooleanSupplier;

/** Class to hold all of the commands for the Yoshi */
public class YoshiCommands {
  private static Command currentCommand = null;

  private static LoggedTunableNumber pivotSetter =
      new LoggedTunableNumber("Yoshivator/Pivot/Debugging/SetpointDegrees", 0.0);

  private static Rotation2d pivotSetpoint = null;

  private YoshiCommands() {}

  /** Returns a command to run the pivot given a direction */
  public static Command runPivotManual(Yoshivator robotYoshi, YoshiPivotDirection direction) {
    currentCommand =
        Commands.run(() -> robotYoshi.setPivotVolts(direction.getVolts()), robotYoshi)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to run the pivot to a given setpoint */
  public static Command runPivot(Yoshivator robotYoshi) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  pivotSetpoint = Rotation2d.fromDegrees(pivotSetter.get());
                },
                robotYoshi)
            .andThen(
                Commands.runOnce(() -> robotYoshi.setPivotSetpoint(pivotSetpoint), robotYoshi));

    return currentCommand;
  }

  /** Returns a command to run the pivot to a given setpoint */
  public static Command runPivotSetpoint(Yoshivator robotYoshi, YoshiPivotSetpoint setpoint) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  pivotSetpoint = Rotation2d.fromDegrees(setpoint.getPositionDegrees());
                },
                robotYoshi)
            .andThen(
                Commands.runOnce(() -> robotYoshi.setPivotSetpoint(pivotSetpoint), robotYoshi));

    return currentCommand;
  }

  /** Returns a command to run the flywheel given a direction */
  public static Command runFlywheelManual(Yoshivator robotYoshi, YoshiFlywheelDirection direction) {
    currentCommand =
        Commands.run(() -> robotYoshi.setFlywheelVolts(direction.getVolts()), robotYoshi)
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to run the pivot to a given setpoint and intake from ground */
  public static Command runIntake(
      Yoshivator robotYoshi, YoshiPivotSetpoint setpoint, YoshiFlywheelDirection direction) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  pivotSetpoint = Rotation2d.fromDegrees(setpoint.getPositionDegrees());
                  robotYoshi.setPivotSetpoint(pivotSetpoint);
                  robotYoshi.setFlywheelVolts(direction.getVolts());
                }, robotYoshi);
    return currentCommand;
  }

  /** Returns a command to run the pivot to a given setpoint and intake from ground */
  public static Command runIntake(
      Yoshivator robotYoshi,
      YoshiPivotSetpoint setpoint,
      YoshiFlywheelDirection direction,
      BooleanSupplier isPieceStowed) {
    currentCommand =
        new FunctionalCommand(
            () -> {
              pivotSetpoint = Rotation2d.fromDegrees(setpoint.getPositionDegrees());
              robotYoshi.setPivotSetpoint(pivotSetpoint);
              robotYoshi.setFlywheelVolts(direction.getVolts());
            },
            () -> {},
            (interrupted) -> {
              pivotSetpoint = Rotation2d.fromDegrees(YoshiPivotSetpoint.IDLE.getPositionDegrees());
              robotYoshi.stopMotors(true, true);

              robotYoshi.setPivotSetpoint(pivotSetpoint);
            },
            isPieceStowed,
            robotYoshi);

    return currentCommand;
  }

  /** Returns a command to score in the amp */
  public static Command scoreAmp(Yoshivator robotYoshi) {
    Debouncer debouncer = new Debouncer(0.5);

    currentCommand =
        new FunctionalCommand(
            () -> {
              pivotSetpoint = Rotation2d.fromDegrees(YoshiPivotSetpoint.AMP.getPositionDegrees());
              robotYoshi.setPivotSetpoint(pivotSetpoint);
            },
            () -> {
              if (debouncer.calculate(true)) {
                robotYoshi.setFlywheelVolts(YoshiFlywheelDirection.IN.getVolts());
              }
            },
            (interrupted) -> {
              robotYoshi.stopMotors(true, true);
            },
            () -> robotYoshi.isPivotAtGoal(),
            robotYoshi);

    return currentCommand;
  }

  /** Returns a command to stop the Yoshi motors */
  public static Command stopYoshi(Yoshivator robotYoshi, boolean stopPivot, boolean stopFlywheel) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }

    currentCommand =
        Commands.run(() -> robotYoshi.stopMotors(stopPivot, stopFlywheel), robotYoshi)
            .alongWith(
                new InstantCommand(
                    () -> {
                      if (stopPivot) {
                        logDirection(YoshiPivotDirection.STOP);
                        pivotSetpoint = null;
                      }
                      if (stopFlywheel) {
                        logDirection(YoshiFlywheelDirection.STOP);
                      }
                    }));

    return currentCommand;
  }

  /** Write the direction of the Yoshi pivot to console */
  private static void logDirection(YoshiPivotDirection direction) {
    System.out.println("YOSHI PIVOT: " + direction);
  }

  /** Write the direction of the Yoshi flywheel to console */
  private static void logDirection(YoshiFlywheelDirection direction) {
    System.out.println("YOSHI FLYWHEEL: " + direction);
  }

  /** Setpoints of the Yoshi in degrees */
  public static enum YoshiPivotSetpoint {
    /** Setpoint to intake from the ground */
    IDLE(110.0),
    /** Setpoint to score amp */
    AMP(85.0),
    /** Setpoint for when the Yoshi is not in use */
    GROUND(-35.0);

    private double desiredPositionDegrees;

    /** Define the setpoint for the Yoshi pivot */
    YoshiPivotSetpoint(double positionDegrees) {
      this.desiredPositionDegrees = positionDegrees;
    }

    /** Returns the desired position setpoint in degrees */
    public double getPositionDegrees() {
      return this.desiredPositionDegrees;
    }
  }

  /** Direction of the yoshi */
  public static enum YoshiPivotDirection {
    /** Run the Yoshi pivot into the robot */
    IN(2.0),
    /** Run the Yoshi pivot out of the robot */
    OUT(-2.0),
    /** Stop the Yoshi wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Yoshi pivot to spin */
    YoshiPivotDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }

  /** Direction of the yoshi */
  public static enum YoshiFlywheelDirection {
    /** Run the Yoshi wheels in to the robot */
    IN(-7.0),
    /** Run the Yoshi wheels out of the robot */
    OUT(7.0),
    /** Stop the Yoshi wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for the Yoshi flywheels to spin */
    YoshiFlywheelDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }
}
