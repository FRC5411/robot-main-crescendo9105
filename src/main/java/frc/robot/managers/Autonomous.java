// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Manager for autonomous functionalities */
public class Autonomous {
  private Drive robotDrive;

  private Field2d robotField = new Field2d();

  private Command pathFinderCommand = new PrintCommand("Init pathCommand");

  /** Initializes the Autonomous routine manager */
  public Autonomous(Drive drive) {
    robotDrive = drive;
    robotField.setRobotPose(new Pose2d());

    SmartDashboard.putData("Field", robotField);

    robotField.getObject("obstacle").setPose(new Pose2d());
  }

  public void updateRobotFieldPose() {
    robotField.setRobotPose(robotDrive.getPosition());
  }

  public void updatePathFinderObstacles() {
    Pose2d fieldObstaclePose = robotField.getObject("obstacle").getPose();

    List<Pair<Translation2d, Translation2d>> obstacles =
        new ArrayList<Pair<Translation2d, Translation2d>>();
    obstacles.add(
        new Pair<Translation2d, Translation2d>(
            new Translation2d(fieldObstaclePose.getX(), fieldObstaclePose.getX()),
            new Translation2d(fieldObstaclePose.getY(), fieldObstaclePose.getY())));

    Pathfinding.setDynamicObstacles(
        obstacles,
        new Translation2d(robotDrive.getPosition().getX(), robotDrive.getPosition().getRotation()));

    Logger.recordOutput(
        "Obstacles/PT1",
        new Pose2d(
            obstacles.get(0).getFirst().getX(),
            obstacles.get(0).getFirst().getX(),
            new Rotation2d()));
    Logger.recordOutput(
        "Obstacles/PT2",
        new Pose2d(
            obstacles.get(0).getSecond().getY(),
            obstacles.get(0).getSecond().getY(),
            new Rotation2d()));
  }

  public Command getPathFinderCommand(Pose2d desiredPoint) {
    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    pathFinderCommand = AutoBuilder.pathfindToPose(desiredPoint, constraints);

    return pathFinderCommand;
  }
}
