// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.SwerveCommands;
import frc.robot.managers.Autonomous;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;

  private Autonomous autonomousManager;

  private CommandPS4Controller pilotController = new CommandPS4Controller(0);

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
    autonomousManager = new Autonomous(robotDrive);

    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));

    AUTO_CHOOSER =
        new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());

    AUTO_CHOOSER.addDefaultOption("Print Hello", new PrintCommand("Hello"));
    AUTO_CHOOSER.addOption(
        "Pathfind", autonomousManager.getPathFinderCommand(new Pose2d(1, 1, new Rotation2d())));

    SmartDashboard.putNumber("Pathfind: X", 0);
    SmartDashboard.putNumber("Pathfind: Y", 0);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            robotDrive,
            () -> pilotController.getLeftX(),
            () -> -pilotController.getLeftY(),
            () -> pilotController.getRightX()));
    // Reset heading
    pilotController
        .triangle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotDrive.setPose(
                            new Pose2d(
                                robotDrive.getPosition().getTranslation(), new Rotation2d())),
                    robotDrive)
                .ignoringDisable(true)); // Reset even when disabled
    pilotController
        .square()
        .onTrue(pathFind())
        .onFalse(Commands.runOnce(() -> pathFind().cancel(), robotDrive));
  }

  public Command pathFind() {
    double x = SmartDashboard.getNumber("Pathfind: X", 0);
    double y = SmartDashboard.getNumber("Pathfind: Y", 0);

    Logger.recordOutput("PathfindX", x);
    Logger.recordOutput("PathfindY", y);

    Command pathFindCommand =
        autonomousManager.getPathFinderCommand(new Pose2d(x, y, new Rotation2d()));
    return pathFindCommand;
  }

  public Autonomous getAutonomousManager() {
    return autonomousManager;
  }

  // public Command getExamplePathfindCommand() {
  //   Pose2d targetPose = new Pose2d(4.0, 1.0, Rotation2d.fromDegrees(0.0));

  //   PathConstraints constraints =
  //       new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  //   Command pathFindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0, 0.0);

  //   List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
  //   obstacles.add(
  //       new Pair<Translation2d, Translation2d>(new Translation2d(1, 1), new Translation2d(2,
  // 2)));

  //   field.getObject("obstacle").setPose(new Pose2d(obstacles.get(0).getFirst(), new
  // Rotation2d()));

  //   Pose2d fieldObsPos = field.getObject("obstacle").getPose();

  //   obstacles.add(
  //       1,
  //       new Pair<Translation2d, Translation2d>(
  //           new Translation2d(fieldObsPos.getX(), fieldObsPos.getY()),
  //           new Translation2d(fieldObsPos.getY(), fieldObsPos.getX())));

  //   Pathfinding.setDynamicObstacles(
  //       obstacles,
  //       new Translation2d(robotDrive.getPosition().getX(),
  // robotDrive.getPosition().getRotation()));

  //   return pathFindingCommand;
  // }

  public Command getAutonomousCommand() {
    return AUTO_CHOOSER.get();
  }
}
