// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;

/** Class to hold all of the path commands */
public class PathCommands {
  private PathConstraints constraints =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  // private Pose2d desiredPose;

  private Drive robotDrive;

  /** Initialize path commands */
  public PathCommands(Drive drive) {
    robotDrive = drive;
  }

  public Command findPathToPose() {
    // Command pathFindCommand = new PrintCommand("PathFind Command is not initialized");

    return new FunctionalCommand(
        // Init
        () -> {},
        // Execute
        () -> {
          AutoBuilder.pathfindToPose(robotDrive.getPathFinderSetpoint(), constraints).schedule();
        },
        // End
        interrupted -> {
          AutoBuilder.pathfindToPose(robotDrive.getPathFinderSetpoint(), constraints).cancel();
        },
        () -> false,
        robotDrive);
  }
}
