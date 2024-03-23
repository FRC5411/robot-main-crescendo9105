// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
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
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.TargetingSystem;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOSim;
import frc.robot.subsystems.shooter.angler.AnglerIOSparkMax;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOSim;
import frc.robot.subsystems.shooter.launcher.LauncherIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.utils.commands.CommandUtils;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Angler robotShooter;
  private Climb robotClimb;
  private Indexer robotIndexer;
  private Vision robotVision;
  private LEDSubsystem robotLEDs;

  private VisionFuser visionFuser;
  private StateMachine robotStateMachine;
  private DriverCommandFactory driverCommands;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController copilotController = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    robotStateMachine = new StateMachine(robotShooter, robotIntake, robotIndexer, robotClimb);
    driverCommands = new DriverCommandFactory(robotStateMachine);

    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
    try {
      autoChooser =
          new LoggedDashboardChooser<>("Autonomous Selector", CommandUtils.buildAutoChooser());
    } catch (Exception e) {
      autoChooser = new LoggedDashboardChooser<>("Autonomous Selector");
      autoChooser.addDefaultOption(
          "ShootNoteException",
          new SequentialCommandGroup(
              robotStateMachine.setShooterState(ShooterStates.AIM),
              new WaitCommand(1.0),
              robotStateMachine.setIndexerState(IndexerStates.INDEX),
              robotStateMachine.setIntakeState(IntakeStates.OFF)));
    }

    configureTriggers();

    // Use assisted control by default
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
        robotShooter = new Angler(new AnglerIOSparkMax(), new LauncherIOTalonFX());
        robotClimb = new Climb(new ClimbIOSparkMax());
        robotIndexer = new Indexer(new IndexerIOSparkMax());
        robotLEDs = new LEDSubsystem();
        robotVision =
            new Vision(
                new VisionIOPhoton(
                    "LLLeft",
                    new Transform3d(
                        0.12,
                        0.26,
                        0.0,
                        new Rotation3d(
                            VecBuilder.fill(-0.36, Math.toRadians(0.0), Math.toRadians(0.0)))),
                    0.1),
                new VisionIOPhoton(
                    "LLRight",
                    new Transform3d(
                        0.0,
                        -0.2,
                        0.0,
                        new Rotation3d(
                            VecBuilder.fill(0.36, Math.toRadians(0.0), Math.toRadians(0.0)))),
                    0.1));
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
        robotShooter = new Angler(new AnglerIOSim(), new LauncherIOSim());
        robotClimb = new Climb(new ClimbIOSim());
        robotIndexer = new Indexer(new IndexerIOSim());
        robotLEDs = new LEDSubsystem();
        robotVision =
            new Vision(
                new VisionIOPhotonSim(
                    "LLLeft",
                    new Transform3d(
                        0.0,
                        0.0,
                        0.33,
                        new Rotation3d(
                            Math.toRadians(13.2), Math.toRadians(0), Math.toRadians(25.2))),
                    0.1,
                    () -> robotDrive.getOdometryPose()),
                new VisionIOPhotonSim(
                    "LLRight",
                    new Transform3d(
                        0.36 - 0.037 - 0.18 + 0.1,
                        0.29 - 0.28,
                        0.33,
                        new Rotation3d(
                            Math.toRadians(13.2), Math.toRadians(0), Math.toRadians(25.5))),
                    0.1,
                    () -> robotDrive.getOdometryPose()));
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
        robotShooter = new Angler(new AnglerIO() {}, new LauncherIO() {});
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIO() {});
        robotLEDs = new LEDSubsystem();
        robotVision = new Vision(new VisionIO() {}, new VisionIO() {});
        break;
    }

    visionFuser = new VisionFuser(robotDrive, robotVision);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    TargetingSystem.getInstance().setSubsystems(robotDrive, robotVision);
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    NamedCommands.registerCommand(
        "Intake",
        new ParallelCommandGroup(
                robotStateMachine.setIndexerState(IndexerStates.STOW),
                robotStateMachine.setIntakeState(IntakeStates.INTAKE),
                robotStateMachine.setShooterState(ShooterStates.INTAKE))
            .withTimeout(2.0));
    NamedCommands.registerCommand(
        "Shoot",
        new SequentialCommandGroup(
            robotStateMachine.setShooterState(ShooterStates.AIM),
            new WaitCommand(1.0),
            robotStateMachine.setIndexerState(IndexerStates.INDEX),
            robotStateMachine.setIntakeState(IntakeStates.OFF)));

    NamedCommands.registerCommand(
        "Eject",
        new WaitCommand(2)
            .andThen(
                new SequentialCommandGroup(
                    robotStateMachine.setShooterState(ShooterStates.EJECT),
                    robotStateMachine.setIndexerState(IndexerStates.INDEX),
                    robotStateMachine.setIntakeState(IntakeStates.INTAKE))));

    NamedCommands.registerCommand(
        "ShootIdle", robotStateMachine.setShooterState(ShooterStates.IDLE));

    NamedCommands.registerCommand(
        "IntakeOnly", robotStateMachine.setIntakeState(IntakeStates.INTAKE));
  }

  private void configureTriggers() {
    // For LEDS
    new Trigger(
            () ->
                robotShooter.atAllSetpoint()
                    && robotStateMachine.getShooterState() == ShooterStates.AIM)
        .onTrue(new InstantCommand(() -> robotLEDs.setReadyColor()))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (robotIndexer.isBeamBroken()) robotLEDs.setHasPiece();
                  else robotLEDs.setDefaultColor();
                }));

    new Trigger(() -> robotIndexer.isBeamBroken())
        .onTrue(new InstantCommand(() -> robotLEDs.setHasPiece()))
        .onFalse(new InstantCommand(() -> robotLEDs.setDefaultColor()));
  }

  /** Configure controllers */
  private void configureButtonBindings() {
    if (Constants.useDebuggingBindings) {
      robotDrive.setDefaultCommand(
          SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> -pilotController.getRightX()));

      //   pilotController
      //       .a()
      //       .whileTrue(
      //           SwerveCommands.setHeading(
      //               robotDrive,
      //               () -> 0.0,
      //               () -> 0.0,
      //               () -> TargetingSystem.getInstance().getOptimalLaunchHeading()))
      //       .onFalse(SwerveCommands.stopDrive(robotDrive));

      pilotController.a().onTrue(SwerveCommands.resetGyro(robotDrive));

      pilotController
          .b()
          .whileTrue(robotStateMachine.setShooterState(ShooterStates.AIM))
          .onFalse(robotStateMachine.setShooterState(ShooterStates.IDLE));

      pilotController
          .x()
          .onTrue(robotShooter.characterizeFlywheel())
          .onFalse(robotStateMachine.setShooterState(ShooterStates.IDLE));

      pilotController
          .y()
          .onTrue(robotDrive.characterizeDriveMotors())
          .onFalse(SwerveCommands.stopDrive(robotDrive));

    } else {
      /* Pilot bindings */

      /* Drive with joysticks */
      robotDrive.setDefaultCommand(
          SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> -pilotController.getRightX()));

      pilotController
          .leftBumper()
          .whileTrue(driverCommands.intakeNote())
          .onFalse(driverCommands.stopTakeNote());

      pilotController
          .rightBumper()
          .whileTrue(driverCommands.outtakeNote())
          .onFalse(driverCommands.stopTakeNote());

      /* Reset gyro */
      pilotController.y().whileTrue(SwerveCommands.resetGyro(robotDrive));

      /* Auto heading to speaker */
      pilotController
          .a()
          .whileTrue(
              SwerveCommands.setHeading(
                  robotDrive,
                  () -> 0.0,
                  () -> 0.0,
                  () -> TargetingSystem.getInstance().getOptimalLaunchHeading()))
          .onFalse(SwerveCommands.stopDrive(robotDrive));

      //   pilotController
      //       .b()
      //       .onTrue(
      //           new InstantCommand(
      //               () -> robotDrive.setPose(new Pose2d(1.34, 5.54, new Rotation2d()))));

      /* Copilot bindings */

      copilotController
          .povUp()
          .whileTrue(driverCommands.moveAnglerUpManual())
          .onFalse(driverCommands.shooterToIdle());

      copilotController
          .povDown()
          .whileTrue(driverCommands.moveAnglerDownManual())
          .onFalse(driverCommands.shooterToIdle());

      copilotController
          .povLeft()
          .whileTrue(driverCommands.climbUp())
          .whileFalse(driverCommands.stopClimb());

      copilotController
          .povRight()
          .whileTrue(driverCommands.climbDown())
          .whileFalse(driverCommands.stopClimb());

      copilotController
          .y()
          .whileTrue(driverCommands.prepareNoteShot())
          .onFalse(driverCommands.stopShooting());

      copilotController
          .b()
          .whileTrue(driverCommands.revUp())
          .whileFalse(driverCommands.stopShooting());

      copilotController
          .a()
          .whileTrue(driverCommands.revAmp())
          .onFalse(driverCommands.stopShooting());

      copilotController
          .x()
          .whileTrue(driverCommands.scoreAmp())
          .onFalse(robotStateMachine.setIndexerState(IndexerStates.OFF));

      copilotController
          .leftBumper()
          .whileTrue(driverCommands.shootNote())
          .onFalse(driverCommands.stopShooting());

      //   copilotController
      //       .rightTrigger()
      //       .whileTrue(robotStateMachine.feedShot())
      //       .onFalse(robotStateMachine.stopShooting());

      //   copilotController
      //       .leftTrigger()
      //       .whileTrue(robotStateMachine.podiumShot())
      //       .onFalse(robotStateMachine.stopShooting());
    }
  }

  public void initLEDs() {
    robotLEDs.setDefaultColor();
  }

  public void shutOffLEDs() {
    robotLEDs.turnOffLEDs();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public VisionFuser getVisionFuser() {
    return visionFuser;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (robotDrive.getPPRotationTargetOverride()) {
      return Optional.of(TargetingSystem.getInstance().getOptimalLaunchHeading());
    } else {
      return Optional.empty();
    }
  }

  public void reset() {
    robotDrive.resetModules();
  }
}
