// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveCommands;
import frc.robot.managers.CommandDispatcher;
import frc.robot.managers.TargetingSystem;
import frc.robot.managers.VisionFuser;
import frc.robot.managers.RobotDesiredStates.AnglerStates;
import frc.robot.managers.RobotDesiredStates.IndexerStates;
import frc.robot.managers.RobotDesiredStates.IntakeStates;
import frc.robot.managers.RobotDesiredStates.LauncherStates;
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
import frc.robot.subsystems.shooter.Shooter;
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
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIO;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOSparkMax;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Shooter robotShooter;
  private Climb robotClimb;
  private Indexer robotIndexer;
  private Vision robotVision;
  private Yoshivator robotYoshi;
  private LEDSubsystem robotLEDs;

  private VisionFuser visionFuser;
  private CommandDispatcher dispatcher;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController copilotController = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    dispatcher = new CommandDispatcher(robotShooter, robotIntake, robotIndexer, robotClimb, robotYoshi);

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
              dispatcher.getShooterCommand(LauncherStates.SPEAKER_SHOT, AnglerStates.AIM),
              new WaitCommand(1.0),
              dispatcher.getIndexerCommand(IndexerStates.INDEX),
              dispatcher.getIntakeCommand(IntakeStates.OFF)));
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
        robotShooter = new Shooter(new AnglerIOSparkMax(), new LauncherIOTalonFX());
        robotClimb = new Climb(new ClimbIOSparkMax());
        robotIndexer = new Indexer(new IndexerIOSparkMax());
        robotLEDs = new LEDSubsystem();
        robotYoshi = new Yoshivator(new ManipulatorIOSparkMax() {});
        robotVision =
            new Vision(
                new VisionIOPhoton(
                    "LLLeft",
                    new Transform3d(
                        0.35,
                        0.32,
                        0.33,
                        new Rotation3d(
                            Math.toRadians(0), Math.toRadians(-25.5), Math.toRadians(-19.2))),
                    0.1),
                new VisionIOPhoton(
                    "LLRight",
                    new Transform3d(
                        0.35,
                        -0.32,
                        0.33,
                        new Rotation3d(
                            Math.toRadians(0), Math.toRadians(-25.5), Math.toRadians(14.7))),
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
        robotShooter = new Shooter(new AnglerIOSim(), new LauncherIOSim());
        robotClimb = new Climb(new ClimbIOSim());
        robotIndexer = new Indexer(new IndexerIOSim());
        robotLEDs = new LEDSubsystem();
        robotYoshi = new Yoshivator(new ManipulatorIOSim() {});
        robotVision =
            new Vision(
                new VisionIOPhotonSim(
                    "LLLeft",
                    new Transform3d(
                        0.2,
                        0.0,
                        0.33,
                        new Rotation3d(
                            Math.toRadians(13.2), Math.toRadians(0), Math.toRadians(25.2))),
                    0.1,
                    () -> robotDrive.getOdometryPose()),
                new VisionIOPhotonSim(
                    "LLRight",
                    new Transform3d(
                        0.4,
                        0.0,
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
        robotShooter = new Shooter(new AnglerIO() {}, new LauncherIO() {});
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIO() {});
        robotLEDs = new LEDSubsystem();
        robotVision = new Vision(new VisionIO() {}, new VisionIO() {});
        robotYoshi = new Yoshivator(new ManipulatorIO() {});
        break;
    }

    visionFuser = new VisionFuser(robotDrive, robotVision);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    TargetingSystem.getInstance().setSubsystems(robotDrive, robotVision, robotShooter);
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    // NamedCommands.registerCommand(
    //     "StowPiece",
    //     new ParallelCommandGroup(
    //             robotStateMachine.getIndexerCommand(IndexerStates.STOW),
    //             robotStateMachine.getIntakeCommand(IntakeStates.INTAKE),
    //             robotStateMachine.getShooterCommand(ShooterStates.INTAKE))
    //         .withTimeout(2.0));

    // // Add intake off if yo
    // NamedCommands.registerCommand(
    //     "Shoot",
    //     new SequentialCommandGroup(
    //         robotStateMachine.getShooterCommand(ShooterStates.AIM),
    //         new WaitCommand(1.3),
    //         robotStateMachine.getIndexerCommand(IndexerStates.INDEX)));

    // NamedCommands.registerCommand(
    //     "Eject",
    //     new WaitCommand(2)
    //         .andThen(
    //             new SequentialCommandGroup( 
    //                 robotStateMachine.getShooterCommand(ShooterStates.EJECT),
    //                 robotStateMachine.getIndexerCommand(IndexerStates.INDEX),
    //                 robotStateMachine.getIntakeCommand(IntakeStates.INTAKE))));

    // NamedCommands.registerCommand(
    //     "ShootIdle", robotStateMachine.getShooterCommand(ShooterStates.IDLE));

    // NamedCommands.registerCommand(
    //     "IntakeOn", robotStateMachine.getIntakeCommand(IntakeStates.INTAKE));

    // NamedCommands.registerCommand("SpeakerShot", new SequentialCommandGroup(
    //   robotStateMachine.getShooterCommand(ShooterStates.SPEAKER).withTimeout(1.0),
    //   robotStateMachine.getIndexerCommand(IndexerStates.INDEX)));

    // NamedCommands.registerCommand("DeployYoshi", robotStateMachine.getYoshiCommand(YoshiStates.GROUND_INTAKE));

    // NamedCommands.registerCommand("UnDeployYoshi", robotStateMachine.getYoshiCommand(YoshiStates.IDLE));
  }

  private void configureTriggers() {
    // For LEDS
    new Trigger(
            () ->
                robotShooter.atAllSetpoints()
                    && (
                      dispatcher.getAnglerState() == AnglerStates.AIM &&
                      dispatcher.getLauncherState() == LauncherStates.SPEAKER_SHOT))
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

  private void configureButtonBindings() {
    if (Constants.useDebuggingBindings) {
      robotDrive.setDefaultCommand(
          SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> -pilotController.getRightX()));

      pilotController.a().onTrue(SwerveCommands.resetGyro(robotDrive));
    } else {
      /* Pilot bindings */
      robotDrive.setDefaultCommand(
          SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> -pilotController.getRightX()));

      pilotController
          .leftBumper()
          .whileTrue(dispatcher.intakeNote())
          .onFalse(dispatcher.stopTakeNote());

      pilotController
          .rightBumper()
          .whileTrue(dispatcher.outtakeNote())
          .onFalse(dispatcher.stopTakeNote());

      pilotController
          .leftTrigger()
          .whileTrue(dispatcher.yoshiIntakeNote())
          .onFalse(dispatcher.stopTakeNote());

      pilotController.y().onTrue(SwerveCommands.resetGyro(robotDrive));

      pilotController
          .a()
          .whileTrue(
              SwerveCommands.setHeading(
                  robotDrive,
                  () -> 0.0,
                  () -> 0.0,
                  () -> TargetingSystem.getInstance().getOptimalLaunchHeading()))
          .onFalse(SwerveCommands.stopDrive(robotDrive));

      /* Copilot bindings */
      copilotController
          .povUp()
          .whileTrue(dispatcher.moveAnglerUpManual())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .povDown()
          .whileTrue(dispatcher.moveAnglerDownManual())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .povLeft()
          .whileTrue(dispatcher.climbUp())
          .whileFalse(dispatcher.stopClimb());

      copilotController
          .povRight()
          .whileTrue(dispatcher.climbDown())
          .whileFalse(dispatcher.stopClimb());

      copilotController
          .y()
          .whileTrue(dispatcher.prepareNoteShot())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .b()
          .whileTrue(dispatcher.revUp())
          .whileFalse(dispatcher.shooterToIdle());

      copilotController
          .a()
          .whileTrue(dispatcher.revAmp())  
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .x()
          .whileTrue(dispatcher.scoreAmp())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .leftBumper()
          .whileTrue(dispatcher.shootNote())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .rightBumper()
          .whileTrue(dispatcher.feedShot())
          .onFalse(dispatcher.shooterToIdle());

      copilotController
          .leftTrigger()
          .onTrue(TargetingSystem.getInstance().incrementOffset());

      copilotController
          .rightTrigger()
          .onTrue(TargetingSystem.getInstance().decrementOffset());
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
