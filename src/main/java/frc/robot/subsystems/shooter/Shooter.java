// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Angler.AnglerConstants;
import frc.robot.subsystems.shooter.Angler.AnglerIO;
import frc.robot.subsystems.shooter.Angler.AnglerInputsAutoLogged;
import frc.robot.subsystems.shooter.Angler.AnglerNEO;
import frc.robot.subsystems.shooter.Angler.AnglerSim;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelConstants;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelSimIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelTalonFX;
import frc.robot.subsystems.shooter.Indexer.Indexer550;
import frc.robot.subsystems.shooter.Indexer.IndexerIO;
import frc.robot.subsystems.shooter.Indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Indexer.IndexerSim;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterWheelIO shooterWheelTop;
  private ShooterWheelIO shooterWheelBottom;
  private IndexerIO indexerIO;
  private AnglerIO anglerIO;

  // Placeholder subsystems with no functionality, just used to satisfy command requirements
  private Subsystem shooterWheelSub = new Subsystem() {};
  private Subsystem indexerSub = new Subsystem() {};
  private Subsystem anglerSub = new Subsystem() {};

  private double topVelocityMPS = 0;
  private double bottomVelocityMPS = 0;

  private double topVolts = 0;
  private double bottomVolts = 0;

  private double indexerVoltage = 0;

  private ShooterWheelIOInputsAutoLogged shooterWheelIOInputsAutoLoggedTop =
      new ShooterWheelIOInputsAutoLogged();
  private ShooterWheelIOInputsAutoLogged shooterWheelIOInputsAutoLoggedBottom =
      new ShooterWheelIOInputsAutoLogged();
  private AnglerInputsAutoLogged anglerInputsAutoLogged = new AnglerInputsAutoLogged();
  private IndexerIOInputsAutoLogged indexerIOInputsAutoLogged = new IndexerIOInputsAutoLogged();

  public Shooter() {
    if (RobotBase.isReal()) {
      shooterWheelTop =
          new ShooterWheelTalonFX(
              ShooterWheelConstants.kTopMotorID,
              false,
              new PIDController(
                  ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kS, ShooterWheelConstants.kV, ShooterWheelConstants.kA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "TopWheel");
      shooterWheelBottom =
          new ShooterWheelTalonFX(
              ShooterWheelConstants.kBottomMotorID,
              true,
              new PIDController(
                  ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kS, ShooterWheelConstants.kV, ShooterWheelConstants.kA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "BottomWheel");

      indexerIO = new Indexer550();
      anglerIO = new AnglerNEO(AnglerConstants.kMotorID);
    } else if (RobotBase.isSimulation()) {
      shooterWheelTop =
          new ShooterWheelSimIO(
              new PIDController(
                  ShooterWheelConstants.kSimP,
                  ShooterWheelConstants.kSimI,
                  ShooterWheelConstants.kSimD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kSimS,
                  ShooterWheelConstants.kSimV,
                  ShooterWheelConstants.kSimA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "TopWheel");
      shooterWheelBottom =
          new ShooterWheelSimIO(
              new PIDController(
                  ShooterWheelConstants.kSimP,
                  ShooterWheelConstants.kSimI,
                  ShooterWheelConstants.kSimD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kSimS,
                  ShooterWheelConstants.kSimV,
                  ShooterWheelConstants.kSimA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "BottomWheel");

      indexerIO = new IndexerSim();
      anglerIO = new AnglerSim();
    } else {
      shooterWheelTop = new ShooterWheelIO() {};
      shooterWheelBottom = new ShooterWheelIO() {};
      indexerIO = new IndexerIO() {};
      anglerIO = new AnglerIO() {};
    }

    shooterWheelSub.setDefaultCommand(
        (ShooterConstants.isClosedLoop)
            ? shooterVelocityCommand(() -> topVelocityMPS, () -> bottomVelocityMPS)
            : shooterVoltsCommand(() -> topVolts, () -> bottomVolts));

    anglerIO.setGoal(Rotation2d.fromDegrees(15));

    anglerSub.setDefaultCommand(anglerPositionCommand());

    indexerSub.setDefaultCommand(indexerVoltageCommand(() -> indexerVoltage));
  }

  public Command shootToSpeakerCommand(
      Pose2d robotPose, BooleanSupplier isRobotInPosition, double desiredMPS) {
    return new SequentialCommandGroup(
        setShooterVelocitySetpointCommand(
            ShooterWheelConstants.kShootMPS,
            ShooterWheelConstants.kShootMPS,
            Rotation2d.fromDegrees(
                TrajectoryAngleSolver.newtonRaphsonSolver(
                    ShooterConstants.kSpeaker3DPose
                        .toPose2d()
                        .minus(robotPose)
                        .getTranslation()
                        .getNorm(),
                    ShooterWheelConstants.kShootMPS))),
        new WaitUntilCommand(isRobotInPosition),
        new InstantCommand(() -> indexerVoltage = 12.0, this));
  }

  public Command setShooterVelocitySetpointCommand(
      double topVelocityMPSSetpoint,
      double bottomVelocityMPSSetpoint,
      Rotation2d screwAngleSetpoint) {
    return new InstantCommand(
        () -> {
          topVelocityMPS = topVelocityMPSSetpoint;
          bottomVelocityMPS = bottomVelocityMPSSetpoint;

          anglerIO.setGoal(screwAngleSetpoint);
          anglerIO.initPID();
        },
        this);
  }

  public Command setShooterVoltageSetpointCommand(
      double topVoltsSetpoint, double bottomVoltsSetpoint, Rotation2d screwAngleSetpoint) {
    return new InstantCommand(
        () -> {
          topVolts = topVoltsSetpoint;
          bottomVolts = bottomVoltsSetpoint;

          anglerIO.setGoal(screwAngleSetpoint);
          anglerIO.initPID();
        },
        this);
  }

  public Command setIndexerVoltage(double volts) {
    return new InstantCommand(() -> indexerVoltage = volts, this);
  }

  public Command shooterVelocityCommand(
      DoubleSupplier topVelocityMPS, DoubleSupplier bottomVelocityMPS) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooterWheelTop.setFlywheelsVelocity(topVelocityMPS.getAsDouble());
          shooterWheelBottom.setFlywheelsVelocity(bottomVelocityMPS.getAsDouble());
        },
        (interrupted) -> {},
        () -> false,
        shooterWheelSub);
  }

  public Command shooterVoltsCommand(DoubleSupplier topVolts, DoubleSupplier bottomVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooterWheelTop.setFlywheelsVolts(topVolts.getAsDouble());
          shooterWheelBottom.setFlywheelsVolts(bottomVolts.getAsDouble());
        },
        (interrupted) -> {},
        () -> false,
        shooterWheelSub);
  }

  public Command anglerPositionCommand() {
    return new FunctionalCommand(
        () -> anglerIO.initPID(),
        () -> anglerIO.executePID(),
        (interrupted) -> {},
        () -> false,
        anglerSub);
  }

  public Command indexerVoltageCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
        () -> {},
        () -> indexerIO.setIndexerVolts(voltage.getAsDouble()),
        (interrupted) -> {},
        () -> false,
        indexerSub);
  }

  public boolean isShooterAtSetpoint() {
    return Math.abs(
                shooterWheelIOInputsAutoLoggedTop.flywheelVelocityMPSSetpoint
                    - shooterWheelIOInputsAutoLoggedTop.flywheelVelocityMPS)
            < 0.5
        && Math.abs(
                shooterWheelIOInputsAutoLoggedTop.flywheelVelocityMPSSetpoint
                    - shooterWheelIOInputsAutoLoggedTop.flywheelVelocityMPS)
            < 0.5
        && Math.abs(
                anglerInputsAutoLogged.anglerAngleSetpoint.getDegrees()
                    - anglerInputsAutoLogged.anglerAngle.getDegrees())
            < 0.5;
  }

  @Override
  public void periodic() {
    shooterWheelTop.updateInputs(shooterWheelIOInputsAutoLoggedTop);
    shooterWheelBottom.updateInputs(shooterWheelIOInputsAutoLoggedBottom);
    anglerIO.updateInputs(anglerInputsAutoLogged);
    indexerIO.updateInputs(indexerIOInputsAutoLogged);

    Logger.processInputs("Shooter/TopWheel", shooterWheelIOInputsAutoLoggedTop);
    Logger.processInputs("Shooter/BottomWheel", shooterWheelIOInputsAutoLoggedBottom);
    Logger.processInputs("Shooter/Angler", anglerInputsAutoLogged);
    Logger.processInputs("Shooter/Indexer", indexerIOInputsAutoLogged);

    if (DriverStation.isDisabled()) {
      shooterWheelTop.setFlywheelsVolts(0.0);
      shooterWheelBottom.setFlywheelsVolts(0.0);
      indexerIO.setIndexerVolts(0.0);
      anglerIO.setAnglerVolts(0.0);
    }
  }
}
