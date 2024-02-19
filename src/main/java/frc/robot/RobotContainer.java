// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
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
import frc.robot.subsystems.shooter.ProjectileTrajectoryGenerator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOSim;
import frc.robot.subsystems.shooter.angler.AnglerIOSparkMax;
import frc.robot.subsystems.shooter.indexer.IndexerIO;
import frc.robot.subsystems.shooter.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOSim;
import frc.robot.subsystems.shooter.launcher.LauncherIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Shooter robotShooter;
  private Climb robotClimb;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController copilotController = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    // NamedCommands MUST be registered before building the auto chooser
    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
    autoChooser =
        new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Print Hello", new PrintCommand("Hello"));

    // configureButtonBindings();
    configureCompetitionButtonBindings();

    // TODO Remove
    ProjectileTrajectoryGenerator.displayDebuggingInformation();
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
        robotShooter =
            new Shooter(new AnglerIOSparkMax(), new IndexerIOSparkMax(), new LauncherIOTalonFX());
        robotClimb = new Climb(new ClimbIOSparkMax());
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
        robotShooter = new Shooter(new AnglerIOSim(), new IndexerIOSim(), new LauncherIOSim());
        robotClimb = new Climb(new ClimbIOSim());
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
        robotShooter = new Shooter(new AnglerIO() {}, new IndexerIO() {}, new LauncherIO() {});
        robotClimb = new Climb(new ClimbIO() {});
        break;
    }
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    // Register commands with PathPlanner's AutoBuilder so it can call them
    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));
    NamedCommands.registerCommand("Intake", IntakeCommands.intakePiece(robotIntake, 12.0));
    NamedCommands.registerCommand("Stop Intake", IntakeCommands.stopIntake(robotIntake));
  }

  /** Configure competition controllers */
  private void configureCompetitionButtonBindings() {
    /* Drive with joysticks */
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            robotDrive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> -pilotController.getRightX()));

    /* Reset gyro / field-oriented */
    pilotController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotDrive.resetGyro();
                },
                robotDrive));

    /* Intake */
    pilotController
        .leftBumper()
        .whileTrue(
            IntakeCommands.intakePiece(robotIntake, 12.0)
                .alongWith(ShooterCommands.runAll(robotShooter, 0.0, 12.0, 0.0)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(ShooterCommands.stopShooter(robotShooter)));

    /* Outtake */
    pilotController
        .rightBumper()
        .whileTrue(
            IntakeCommands.outtakePiece(robotIntake, -12.0)
                .alongWith(ShooterCommands.runAll(robotShooter, 0.0, -12.0, 0.0)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(ShooterCommands.stopShooter(robotShooter)));

    /* Run the flywheel and then the indexer */
    copilotController
        .y()
        .toggleOnTrue(
            Commands.run(
                    () -> {
                      robotShooter.setManualLauncher(-9.0);
                    },
                    robotShooter)
                .withTimeout(2.0)
                .andThen(
                    Commands.run(
                            () -> {
                              robotShooter.setManualIndexer(12.0);
                              robotIntake.setVolts(12.0);
                            },
                            robotShooter)
                        .withTimeout(1.0)))
        .toggleOnFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualIndexer(0.0);
                  robotShooter.setManualLauncher(0.0);
                  robotIntake.setVolts(0.0);
                },
                robotShooter));

    /* Run launcher */
    copilotController
        .x()
        .toggleOnTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualLauncher(-12.0);
                },
                robotShooter))
        .toggleOnFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualLauncher(0.0);
                },
                robotShooter));

    /* Run indexer */
    copilotController
        .b()
        .toggleOnTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualIndexer(12.0);
                  robotIntake.setVolts(12.0);
                },
                robotShooter))
        .toggleOnFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualIndexer(0.0);
                  robotIntake.setVolts(0.0);
                },
                robotShooter));

    /* Angle up */
    copilotController
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(-8.0);
                },
                robotShooter))
        .whileFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(0.0);
                },
                robotShooter));

    /* Angle down */
    copilotController
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(8.0);
                },
                robotShooter))
        .whileFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(0.0);
                },
                robotShooter));
  }

  /** Configure controllers */
  public void configureButtonBindings() {
    /* Drive with joysticks */
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            robotDrive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> -pilotController.getRightX()));

    /* Reset drive heading | Debugging */
    pilotController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotDrive.resetGyro();
                },
                robotDrive));

    /* Reset drive pose | Debugging */
    pilotController.a().onTrue(Commands.runOnce(robotDrive::resetPose, robotDrive));

    /* Run the flywheel and then the indexer */
    pilotController
        .b()
        .toggleOnTrue(
            Commands.run(
                    () -> {
                      robotShooter.setManualLauncher(-9.0);
                    },
                    robotShooter)
                .withTimeout(2.0)
                .andThen(
                    Commands.run(
                            () -> {
                              robotShooter.setManualIndexer(12.0);
                              robotIntake.setVolts(12.0);
                            },
                            robotShooter)
                        .withTimeout(1.0)))
        .toggleOnFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualIndexer(0.0);
                  robotShooter.setManualLauncher(0.0);
                  robotIntake.setVolts(0.0);
                },
                robotShooter));

    /* Angle up */
    pilotController
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(-8.0);
                },
                robotShooter))
        .whileFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(0.0);
                },
                robotShooter));

    /* Angle down */
    pilotController
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(8.0);
                },
                robotShooter))
        .whileFalse(
            Commands.run(
                () -> {
                  robotShooter.setManualAngler(0.0);
                },
                robotShooter));

    /* Intake */
    pilotController
        .leftBumper()
        .whileTrue(
            IntakeCommands.intakePiece(robotIntake, 12.0)
                .alongWith(ShooterCommands.runAll(robotShooter, 0.0, 12.0, 0.0)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(ShooterCommands.stopShooter(robotShooter)));

    /* Outtake */
    pilotController
        .rightBumper()
        .whileTrue(
            IntakeCommands.outtakePiece(robotIntake, -12.0)
                .alongWith(ShooterCommands.runAll(robotShooter, 0.0, -12.0, 0.0)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(ShooterCommands.stopShooter(robotShooter)));

    /* Index */
    pilotController
        .povLeft()
        .whileTrue(ShooterCommands.runAll(robotShooter, 0.0, 12.0, 0.0))
        .whileFalse(ShooterCommands.stopShooter(robotShooter));

    /* Outdex */
    pilotController
        .povRight()
        .whileTrue(ShooterCommands.runAll(robotShooter, 0.0, -12.0, 0.0))
        .whileFalse(ShooterCommands.stopShooter(robotShooter));

    /* Ascend climb */
    pilotController
        .leftTrigger()
        .whileTrue(
            ClimbCommands.runManual(robotClimb, 12.0, 12.0, pilotController.x().getAsBoolean()))
        .whileFalse(ClimbCommands.stopClimb(robotClimb));

    /* Descend climb */
    pilotController
        .rightTrigger()
        .whileTrue(
            ClimbCommands.runManual(robotClimb, -12.0, -12.0, pilotController.x().getAsBoolean()))
        .whileFalse(ClimbCommands.stopClimb(robotClimb));

    /* Test closed loop button */
    // pilotController
    //     .x()
    //     .whileTrue(ShooterCommands.runFlywheels(robotShooter, 2.0))
    //     .whileFalse(ShooterCommands.stopShooter(robotShooter));
  }

  /** Returns the selected autonomous */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
