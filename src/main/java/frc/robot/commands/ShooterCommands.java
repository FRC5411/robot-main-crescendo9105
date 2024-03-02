// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.TargetingSystem;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to hold all of the commands for the Shooter */
public class ShooterCommands {
  private static Command currentCommand = null;

  private static LoggedTunableNumber angleSetter =
      new LoggedTunableNumber("Shooter/Angler/Debugging/SetpointDegrees", 35.0);

  private static LoggedTunableNumber launcherSetter =
      new LoggedTunableNumber("Shooter/Launcher/Debugging/SetpointMPS", 10.0);

  private static Rotation2d anglerPosition = null;
  private static double laucnherVelocityMPS = 0.0;

  private ShooterCommands() {}

  /** Returns a command to run the angler motor */
  public static Command runAngler(Shooter robotShooter) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  anglerPosition = Rotation2d.fromDegrees(angleSetter.get());
                },
                robotShooter)
            .andThen(
                Commands.run(() -> robotShooter.setAllMotors(anglerPosition, 0.0), robotShooter));

    return currentCommand;
  }

  /** Returns a command to run the angler motor */
  public static Command runAngler(Shooter robotShooter, Rotation2d anglerSetpoint) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  anglerPosition = anglerSetpoint;
                },
                robotShooter)
            .andThen(
                Commands.run(
                    () -> robotShooter.setAllMotors(anglerPosition, laucnherVelocityMPS),
                    robotShooter));

    return currentCommand;
  }

  /** Returns a command to run the angler motor */
  public static Command runAngler(Shooter robotShooter, Drive robotDrive) {
    currentCommand =
        new FunctionalCommand(
            () -> {
              TargetingSystem targetingSystem = new TargetingSystem();
              anglerPosition = targetingSystem.getLaunchMapAngle(robotDrive.getPosition());
            },
            () -> {
              robotShooter.setAllMotors(anglerPosition, laucnherVelocityMPS);
              System.out.println("TARGETING SYSTEM MAP: " + anglerPosition);
            },
            interrupted -> {
              robotShooter.stopMotors(true, false);
            },
            () -> false,
            robotShooter);

    return currentCommand;
  }

  /** Returns a command to run the angler manually */
  public static Command runAnglerManual(Shooter robotShooter, AnglerDirection direction) {
    currentCommand =
        Commands.runOnce(() -> robotShooter.setAnglerPosition(null), robotShooter)
            .andThen(
                Commands.runOnce(
                    () -> robotShooter.setAnglerVolts(direction.getVolts()), robotShooter))
            .alongWith(new InstantCommand(() -> logDirection(direction)));

    return currentCommand;
  }

  /** Returns a command to run the launcher motors */
  public static Command runLauncher(Shooter robotsShooter) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  laucnherVelocityMPS = launcherSetter.get();
                },
                robotsShooter)
            .andThen(
                Commands.run(
                    () -> robotsShooter.setAllMotors(anglerPosition, laucnherVelocityMPS),
                    robotsShooter));

    return currentCommand;
  }

  /** Returns a command to run the launcher motors */
  public static Command runLauncher(Shooter robotShooter, double launcherVelocityMPSSetpoint) {
    currentCommand =
        Commands.runOnce(
                () -> {
                  laucnherVelocityMPS = launcherVelocityMPSSetpoint;
                },
                robotShooter)
            .andThen(
                Commands.run(
                    () -> robotShooter.setAllMotors(anglerPosition, laucnherVelocityMPS),
                    robotShooter));

    return currentCommand;
  }

  /** Returns a command to run the launcher manually */
  public static Command runLauncherManual(Shooter robotShooter, FlywheelSpeeds speeds) {
    currentCommand =
        Commands.runOnce(() -> robotShooter.setLauncherVelocityMPS(null), robotShooter)
            .andThen(
                Commands.runOnce(
                    () -> robotShooter.setLauncherVolts(speeds.getVolts(), speeds.getVolts()),
                    robotShooter))
            .alongWith(new InstantCommand(() -> logSpeeds(speeds)));

    return currentCommand;
  }

  /** Returns a command to stop all Shooter motors */
  public static Command stopShooter(
      Shooter robotShooter, boolean stopAngler, boolean stopLauncher) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }

    currentCommand =
        Commands.run(() -> robotShooter.stopMotors(stopAngler, stopLauncher), robotShooter)
            .alongWith(
                new InstantCommand(
                    () -> {
                      if (stopAngler) {
                        logDirection(AnglerDirection.STOP);
                        anglerPosition = null;
                      }
                      if (stopLauncher) {
                        logSpeeds(FlywheelSpeeds.STOP);
                        laucnherVelocityMPS = 0.0;
                      }
                    }));

    return currentCommand;
  }

  /** Write the direction of the Angler to console */
  private static void logDirection(AnglerDirection direction) {
    System.out.println("ANGLER: " + direction);
  }

  /** Write the speeds of the Launcher to console */
  private static void logSpeeds(FlywheelSpeeds speeds) {
    System.out.println("LAUNCHER: " + speeds);
  }

  /** Direction of the Angler */
  public static enum AnglerDirection {
    /** Run the Angler upwards */
    UP(9.0),
    /** Run the Angler downwards */
    DOWN(-9.0),
    /** Stop the Intake wheels */
    STOP(0.0);

    private double volts;

    /** Define the direction for Angler to go */
    AnglerDirection(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the direction specified */
    public double getVolts() {
      return this.volts;
    }
  }

  /** Speed of the Launcher */
  public static enum FlywheelSpeeds {
    /** Run the Launcher wheels at 25% speed */
    QUARTER(3.0),
    /** Run the Launcher wheels at 50% speed */
    HALF(6.0),
    /** Run the Launcher wheels at 75% speed */
    SUBQUARTER(9.0),
    /** Run the Launcher wheels at 100% speed */
    FULL(12.0),
    /** Stop the Launcher wheels */
    STOP(0.0);

    private double volts;

    /** Define the voltage for the Laucnher */
    FlywheelSpeeds(double volts) {
      this.volts = volts;
    }

    /** Returns the voltage based on the speeds specified */
    public double getVolts() {
      return this.volts;
    }
  }
}
