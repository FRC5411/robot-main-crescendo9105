// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;

  private CommandXboxController pilotController = new CommandXboxController(0);
  // private CommandPS4Controller pilotController = new CommandPS4Controller(0);

  private final LoggedDashboardChooser<Command> AUTO_CHOOSER;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        robotDrive =
            new Drive(
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new GyroIOPigeon2(false));
        break;
      case SIM:
        robotDrive =
            new Drive(
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3),
                new GyroIO() {});
        break;
      default:
        robotDrive =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));

    AUTO_CHOOSER =
        new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());

    AUTO_CHOOSER.addDefaultOption("Print Hello", new PrintCommand("Hello"));
    AUTO_CHOOSER.addOption(
        "12V Drive Test",
        Commands.runOnce(
                () -> {
                  robotDrive.setDriveVolts(12.0);
                },
                robotDrive)
            .repeatedly()
            .withTimeout(10.0));
    AUTO_CHOOSER.addOption(
        "6V Drive Test",
        Commands.runOnce(
                () -> {
                  robotDrive.setDriveVolts(6.0);
                },
                robotDrive)
            .repeatedly()
            .withTimeout(10.0));

    

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            // FIXME Figure out why joysticks are being goofy
            robotDrive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> pilotController.getRightX()));
    // Reset heading
    pilotController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotDrive.setPose(
                            new Pose2d(
                                robotDrive.getPosition().getTranslation(), new Rotation2d())),
                    robotDrive)
                .ignoringDisable(true)); // Reset even when disabled
    // pilotController
    //     .a()
    //     .whileTrue(
    //         AutoBuilder.pathfindToPose(
    //             robotDrive.getPathFinderSetpoint(), robotDrive.getPathConstraints()));
  }

  public Command getAutonomousCommand() {
    return AUTO_CHOOSER.get();
  }
}
