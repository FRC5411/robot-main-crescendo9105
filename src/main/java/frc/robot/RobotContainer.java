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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooterrefactored.Shooter;
import frc.robot.subsystems.shooterrefactored.angler.AnglerIO;
import frc.robot.subsystems.shooterrefactored.angler.AnglerIOSim;
import frc.robot.subsystems.shooterrefactored.angler.AnglerIOSparkMax;
import frc.robot.subsystems.shooterrefactored.indexer.IndexerIO;
import frc.robot.subsystems.shooterrefactored.indexer.IndexerIOSim;
import frc.robot.subsystems.shooterrefactored.indexer.IndexerIOSparkMax;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Shooter robotShooter;

  private CommandXboxController pilotController = new CommandXboxController(0);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
    autoChooser =
        new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  /** Instantiate subsystems */
  private void initializeSubsystems() {
    switch (Constants.currentMode) {
      case REAL:
        robotDrive =
            new Drive(
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new GyroIOPigeon2(false));
        robotIntake = new Intake(new IntakeIOSparkMax());
        robotShooter = new Shooter(new AnglerIOSparkMax(), new IndexerIOSparkMax());
        break;
      case SIM:
        robotDrive =
            new Drive(
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3),
                new GyroIO() {});
        robotIntake = new Intake(new IntakeIOSim());
        robotShooter = new Shooter(new AnglerIOSim(), new IndexerIOSim());
        break;
      default:
        robotDrive =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        robotIntake = new Intake(new IntakeIO() {});
        robotShooter = new Shooter(new AnglerIO() {}, new IndexerIO() {});
        break;
    }
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    Logger.recordOutput("Command Running", "Not Running");
    // Register commands with PathPlanner's AutoBuilder so it can call them
    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));
    NamedCommands.registerCommand("Intake", IntakeCommands.intakePiece(robotIntake, 12.0));
    NamedCommands.registerCommand("Stop Intake", IntakeCommands.stopIntake(robotIntake));

    // autoChooser.addDefaultOption("Print Hello", new PrintCommand("Hello"));
  }

  /** Configure controllers */
  private void configureButtonBindings() {
    /* Drive with joysticks */
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            // FIXME Figure out why joysticks are being goofy
            robotDrive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> -pilotController.getRightX()));

    /* Reset drive heading | Debugging */
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

    pilotController
        .a()
        .whileTrue(Commands.run(() -> robotShooter.setIndexerVolts(12.0), robotShooter))
        .whileFalse(Commands.run(() -> robotShooter.setIndexerVolts(0.0), robotShooter));
    pilotController
        .b()
        .whileTrue(Commands.run(() -> robotShooter.setIndexerVolts(-12.0), robotShooter))
        .whileFalse(Commands.run(() -> robotShooter.setIndexerVolts(0.0), robotShooter));

    // /* Reset drive pose | Debugging */
    // pilotController.a().onTrue(Commands.runOnce(robotDrive::resetPose, robotDrive));

    // // /* Run intake (NEO) at half speed */
    // // pilotController
    // //     .b()
    // //     //        .whileTrue(IntakeCommands.runIntake(robotIntake, 5676.0 / 2.0))
    // //     .whileTrue(IntakeCommands.runIntake(robotIntake, 1500.0))
    // //     .whileFalse(IntakeCommands.stopIntake(robotIntake));
    // pilotController
    //     .leftBumper()
    //     .whileTrue(IntakeCommands.intakePiece(robotIntake, 12.0))
    //     .whileFalse(IntakeCommands.stopIntake(robotIntake));
    // pilotController
    //     .rightBumper()
    //     .whileTrue(IntakeCommands.intakePiece(robotIntake, -12.0))
    //     .whileFalse(IntakeCommands.stopIntake(robotIntake));
    // // pilotController
    // //     .b()
    // //     .toggleOnTrue(
    // //         IntakeCommands.intakePiece(robotIntake, 12.0)
    // //             .finallyDo(() -> robotIntake.setVolts(0.0)));

    // pilotController
    //     .a()
    //     .whileTrue(
    //         robotShooter
    //             .setShooterVoltageSetpointCommand(12, 12)
    //             .andThen(robotShooter.setIndexerVoltage(12)))
    //     .onFalse(
    //         robotShooter
    //             .setShooterVoltageSetpointCommand(0, 0)
    //             .andThen(robotShooter.setIndexerVoltage(0)));
    // pilotController
    //     .b()
    //     .onTrue(robotShooter.setIndexerVoltage(-12))
    //     .onFalse(robotShooter.setIndexerVoltage(0));
    // pilotController
    //     .x()
    //     .onTrue(robotShooter.getSysIDTests())
    //     .onFalse(robotShooter.setShooterVoltageSetpointCommand(0, 0));

    // pilotController
    //     .povUp()
    //     .whileTrue(robotShooter.setShooterVelocitySetpointCommand(5, 5))
    //     .whileTrue(
    //         robotShooter.setShooterAngleCommand(
    //             new Rotation2d(TrajectoryAngleSolver.newtonRaphsonSolver(3, 5))))
    //     .onFalse(robotShooter.setShooterVelocitySetpointCommand(0, 0));

    // pilotController
    //     .y()
    //     .onTrue()
    //     .onFalse();

    // .toggleOnFalse(IntakeCommands.stopIntake(robotIntake));

    // /* Set climb to angle */
    // pilotController
    //     .a()
    //     .whileTrue(ClimbCommands.setAngle(robotClimb, 1.0, 1.0))
    //     .whileFalse(ClimbCommands.setAngle(robotClimb, 0.0, 0.0));

    // /* Print commands for debugging purposes */
    // pilotController
    //     .b()
    //     .whileTrue(Commands.print("B Button | whileTrue"))
    //     .whileFalse(Commands.print("B Button | whileFalse"));
    // pilotController.a().whileTrue(Commands.print("A Button | whileTrue"));
    // pilotController.y().whileTrue(Commands.print("Y Button | whileTrue"));
    // pilotController.x().whileTrue(Commands.print("X Button | whileTrue"));
  }

  /** Returns the selected autonomous */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
