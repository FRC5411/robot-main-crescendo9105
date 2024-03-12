// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IndexerCommands.IndexerDirection;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.IntakeCommands.IntakeDirection;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.YoshiCommands;
import frc.robot.commands.YoshiCommands.YoshiFlywheelDirection;
import frc.robot.commands.YoshiCommands.YoshiPivotSetpoint;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
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
import frc.robot.subsystems.shooter.Shooter.AnglerSetpoints;
import frc.robot.subsystems.shooter.Shooter.LauncherSetpoints;
import frc.robot.subsystems.shooter.TargetingSystem;
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
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIO;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOSparkMax;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive robotDrive;
  private Intake robotIntake;
  private Shooter robotShooter;
  private Climb robotClimb;
  private Indexer robotIndexer;
  private Yoshivator robotYoshi;
  private Vision robotVision;
  private LEDSubsystem robotLEDs;

  private VisionFuser visionFuser;
  private StateMachine stateMachine;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController copilotController = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    stateMachine =
        new StateMachine(robotShooter, robotIntake, robotIndexer, robotYoshi, robotClimb);

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
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIOSparkMax());
        robotYoshi = new Yoshivator(new ManipulatorIOSparkMax());
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
        robotShooter = new Shooter(new AnglerIOSim(), new LauncherIOSim());
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIOSim());
        robotYoshi = new Yoshivator(new ManipulatorIOSim());
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
        robotShooter = new Shooter(new AnglerIO() {}, new LauncherIO() {});
        robotClimb = new Climb(new ClimbIO() {});
        robotIndexer = new Indexer(new IndexerIO() {});
        robotYoshi = new Yoshivator(new ManipulatorIO() {});
        robotLEDs = new LEDSubsystem();
        robotVision = new Vision(new VisionIO() {}, new VisionIO() {});
        break;
    }

    visionFuser = new VisionFuser(robotDrive, robotVision);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    // Register commands with PathPlanner's AutoBuilder so it can call them
    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPoseEstimate()));

    NamedCommands.registerCommand(
        "AutoShoot",
        IndexerCommands.stopIndexer(robotIndexer)
            .alongWith(IntakeCommands.stopIntake(robotIntake))
            .alongWith(ShooterCommands.automaticTarget(robotShooter).withTimeout(3.0)));

    NamedCommands.registerCommand(
        "FirePiece",
        IntakeCommands.runIntake(robotIntake, IntakeDirection.IN)
            .alongWith(IndexerCommands.runIndexer(robotIndexer, IndexerDirection.IN))
            .withTimeout(3.0));

    NamedCommands.registerCommand(
        "IntakeInnerRollers",
        ShooterCommands.runAll(robotShooter, Rotation2d.fromDegrees(40.0), 0.0)
            .alongWith(IntakeCommands.runIntake(robotIntake, IntakeDirection.IN))
            .alongWith(IndexerCommands.stowPiece(robotIndexer))
            .withTimeout(0.1)
            .andThen(IndexerCommands.runIndexer(robotIndexer, IndexerDirection.STOP)));

    NamedCommands.registerCommand(
        "DeployYoshi",
        YoshiCommands.runIntake(robotYoshi, YoshiPivotSetpoint.GROUND, YoshiFlywheelDirection.IN)
            .withTimeout(3.0));

    NamedCommands.registerCommand(
        "RetractYoshi",
        YoshiCommands.runIntake(robotYoshi, YoshiPivotSetpoint.IDLE, YoshiFlywheelDirection.STOP)
            .withTimeout(3.0));

    NamedCommands.registerCommand(
        "AnglerDown", ShooterCommands.runAll(robotShooter, Rotation2d.fromDegrees(35.0), 0.0));
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

      pilotController
          .a()
          .whileTrue(
              SwerveCommands.setHeading(
                  robotDrive,
                  () -> 0.0,
                  () -> 0.0,
                  () -> TargetingSystem.getOptimalLaunchHeading()))
          .onFalse(SwerveCommands.stopDrive(robotDrive));

      pilotController
          .y()
          .whileTrue(
              ShooterCommands.runAngler(robotShooter)
                  .alongWith(
                      TargetingSystem.shoot(
                          () -> robotDrive.getOdometryPose(),
                          () -> robotShooter.getAnglerPosition())))
          .whileFalse(ShooterCommands.stopShooter(robotShooter, true, false));
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
          .a()
          .whileTrue(
              robotShooter.setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.SPEAKER_SHOT))
          .whileFalse(Commands.runOnce(() -> robotShooter.stopMotors(true, true), robotShooter));

      //   /* Intake a note from the ground */
      //   pilotController
      //       .leftBumper()
      //       .whileTrue(
      //           caster
      //               .getCommand(CasterState.INTAKE_GROUND)
      //               .alongWith(superstructure.getCommand(SuperstructureState.INTAKING)))
      //       .whileFalse(
      //           caster
      //               .getCommand(CasterState.IDLE)
      //               .alongWith(superstructure.getCommand(SuperstructureState.IDLE)));

      //   /* Outtake a gamepiece */
      //   pilotController
      //       .rightBumper()
      //       .whileTrue(caster.getCommand(CasterState.OUTTAKE))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Intake a note from the ground into the yoshi */
      //   pilotController
      //       .leftTrigger()
      //       .whileTrue(caster.getCommand(CasterState.INTAKE_AMP))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Reset gyro */
      //   pilotController.y().onTrue(SwerveCommands.resetGyro(robotDrive));

      //   /* Auto heading to speaker */
      //   pilotController
      //       .a()
      //       .whileTrue(
      //           SwerveCommands.setHeading(
      //               robotDrive,
      //               () -> 0.0,
      //               () -> 0.0,
      //               () -> TargetingSystem.getOptimalLaunchHeading()))
      //       .onFalse(SwerveCommands.stopDrive(robotDrive));

      //   /* Copilot bindings */

      //   /* Align yoshi to score in amp */
      //   copilotController
      //       .leftBumper()
      //       .whileTrue(caster.getCommand(CasterState.OUTTAKE))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Prepare to climb */
      //   copilotController
      //       .rightTrigger()
      //       .whileTrue(superstructure.getCommand(SuperstructureState.MANUAL_ANGLER_DOWN))
      //       .whileFalse(superstructure.getCommand(SuperstructureState.IDLE));

      //   /* Climb to setpoint */
      //   copilotController
      //       .leftTrigger()
      //       .whileTrue(superstructure.getCommand(SuperstructureState.MANUAL_ANGLER_UP))
      //       .whileFalse(superstructure.getCommand(SuperstructureState.IDLE));

      //   /* Index note */
      //   copilotController
      //       .povLeft()
      //       .whileTrue(caster.getCommand(CasterState.INDEX))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Outdex note */
      //   copilotController
      //       .povRight()
      //       .whileTrue(caster.getCommand(CasterState.OUTDEX))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Prepare to fire a note at the speaker */
      //   copilotController
      //       .y()
      //       .whileTrue(superstructure.getCommand(SuperstructureState.PREPARING_SHOT))
      //       .whileFalse(superstructure.getCommand(SuperstructureState.IDLE));

      //   /* Align yoshi to score in amp */
      //   copilotController
      //       .a()
      //       .whileTrue(caster.getCommand(CasterState.SCORE_AMP))
      //       .whileFalse(caster.getCommand(CasterState.IDLE));

      //   /* Test climb manual */
      //   copilotController
      //       .x()
      //       .whileTrue(
      //           ClimbCommands.runClimbManual(
      //               robotClimb, ClimbLeftDirection.IN, ClimbRightDirection.OUT))
      //       .whileFalse(ClimbCommands.stopClimb(robotClimb));

      //   /* Test manual climb flip direction */
      //   copilotController.b().onTrue(superstructure.swapClimbDirection());

      //   /* Test manual climb left */
      //   copilotController
      //       .povUp()
      //       .whileTrue(superstructure.getCommand(SuperstructureState.MANUAL_CLIMB_LEFT))
      //       .whileFalse(superstructure.getCommand(SuperstructureState.IDLE));

      //   /* Test manual climb right */
      //   copilotController
      //       .povDown()
      //       .whileTrue(superstructure.getCommand(SuperstructureState.MANUAL_CLIMB_RIGHT))
      //       .whileFalse(superstructure.getCommand(SuperstructureState.IDLE));

      //   copilotController.rightBumper().onTrue(robotLEDs.lightDarkBlueGradientCommand(true));
    }
  }

  /** Returns the selected autonomous */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public VisionFuser getVisionFuser() {
    return visionFuser;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (robotDrive.getPPRotationTargetOverride()) {
      return Optional.of(TargetingSystem.getOptimalLaunchHeading());
    } else {
      return Optional.empty();
    }
  }
}
