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
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IndexerCommands.IndexerDirection;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.IntakeCommands.IntakeDirection;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.AnglerDirection;
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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOSim;
import frc.robot.subsystems.shooter.angler.AnglerIOSparkMax;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOSim;
import frc.robot.subsystems.shooter.launcher.LauncherIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Shooter robotShooter;
  private Climb robotClimb;
  private Indexer robotIndexer;

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
        robotShooter = new Shooter(new AnglerIOSparkMax(), new LauncherIOTalonFX());
        robotClimb = new Climb(new ClimbIOSparkMax());
        robotIndexer = new Indexer(new IndexerIOSparkMax());
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
        robotShooter = new Shooter(new AnglerIOSim(), new LauncherIOSim());
        robotClimb = new Climb(new ClimbIOSim());
        robotIndexer = new Indexer(new IndexerIOSim());
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
        robotShooter = new Shooter(new AnglerIO() {}, new LauncherIO() {});
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIO() {});
        break;
    }
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    // Register commands with PathPlanner's AutoBuilder so it can call them
    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));
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

    // /* Reset gyro */
    // pilotController.y().onTrue(Commands.runOnce(() -> robotDrive.resetGyro(), robotDrive));

    /* Run intake */
    pilotController
        .leftBumper()
        .whileTrue(
            IntakeCommands.runIntake(robotIntake, IntakeDirection.IN)
                .alongWith(IndexerCommands.stowPiece(robotIndexer)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(IndexerCommands.stopIndexer(robotIndexer)));

    /* Run outtake */
    pilotController
        .rightBumper()
        .whileTrue(
            IntakeCommands.runIntake(robotIntake, IntakeDirection.OUT)
                .alongWith(IndexerCommands.runIndexer(robotIndexer, IndexerDirection.OUT)))
        .whileFalse(
            IntakeCommands.stopIntake(robotIntake)
                .alongWith(IndexerCommands.stopIndexer(robotIndexer)));

    /* Run angler setpoint */
    pilotController
        .x()
        .whileTrue(ShooterCommands.runAngler(robotShooter))
        .whileFalse(ShooterCommands.stopShooter(robotShooter, true, true));

    /* Run angler manual up */
    pilotController
        .povUp()
        .whileTrue(ShooterCommands.runAnglerManual(robotShooter, AnglerDirection.UP))
        .whileFalse(ShooterCommands.stopShooter(robotShooter, true, false));

    /* Run angler manual down */
    pilotController
        .povDown()
        .whileTrue(ShooterCommands.runAnglerManual(robotShooter, AnglerDirection.DOWN))
        .whileFalse(ShooterCommands.stopShooter(robotShooter, true, false));

    // /* Run launcher setpoint */
    pilotController
        .b()
        .whileTrue(ShooterCommands.runLauncher(robotShooter))
        .whileFalse(ShooterCommands.stopShooter(robotShooter, false, true));

    // /* Run launcher manual */
    // pilotController
    //     .a()
    //     .whileTrue(IndexerCommands.runIndexer(robotIndexer, IndexerDirection.IN))
    //     .whileFalse(IndexerCommands.stopIndexer(robotIndexer));

    /* Move back slightly */
    pilotController
        .povLeft()
        .whileTrue(SwerveCommands.swerveDrive(robotDrive, () -> -0.3, () -> 0.0, () -> 0.0))
        .onFalse(SwerveCommands.stopDrive(robotDrive));

    /* Move forward slightly */
    pilotController
        .povRight()
        .whileTrue(SwerveCommands.swerveDrive(robotDrive, () -> 0.3, () -> 0.0, () -> 0.0))
        .onFalse(SwerveCommands.stopDrive(robotDrive));

    pilotController
        .y()
        .whileTrue(ShooterCommands.runAngler(robotShooter, robotDrive.getPosition()))
        .whileFalse(ShooterCommands.stopShooter(robotShooter, true, false));

    pilotController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> robotDrive.setPose(new Pose2d(15.05, 5.81, new Rotation2d())), robotDrive));
  }

  /** Returns the selected autonomous */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
