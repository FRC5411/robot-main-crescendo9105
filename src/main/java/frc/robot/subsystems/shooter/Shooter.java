// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.Angler.AnglerInputsAutoLogged;
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
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterWheelIO shooterWheelTop;
  private ShooterWheelIO shooterWheelBottom;
  private IndexerIO indexerIO;
  // private AnglerIO anglerIO;

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

  private boolean runShooter = false;
  private boolean runClosedLoop = false;

  public Shooter() {
    if (RobotBase.isReal()) {
      shooterWheelTop =
          new ShooterWheelTalonFX(
              ShooterWheelConstants.kTopMotorID,
              true,
              new PIDController(
                  ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kS, ShooterWheelConstants.kV, ShooterWheelConstants.kA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "TopWheel");
      shooterWheelBottom =
          new ShooterWheelTalonFX(
              ShooterWheelConstants.kBottomMotorID,
              false,
              new PIDController(
                  ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD),
              new SimpleMotorFeedforward(
                  ShooterWheelConstants.kS, ShooterWheelConstants.kV, ShooterWheelConstants.kA),
              ShooterWheelConstants.kFlywheelRateLimit,
              "BottomWheel");

      indexerIO = new Indexer550();
      // anglerIO = new AnglerNEO(AnglerConstants.kMotorID);
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
      // anglerIO = new AnglerSim();
    } else {
      shooterWheelTop = new ShooterWheelIO() {};
      shooterWheelBottom = new ShooterWheelIO() {};
      indexerIO = new IndexerIO() {};
      // anglerIO = new AnglerIO() {};
    }

    // anglerIO.setGoal(Rotation2d.fromDegrees(15));
    // anglerIO.initPID();
  }

  public Command shootToSpeakerCommand(
      Pose2d robotPose, BooleanSupplier isRobotInPosition, double desiredMPS) {
    return new SequentialCommandGroup(
        setShooterSepointsCommand(
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

  public Command setShooterSepointsCommand(
      double topVelocityMPSSetpoint, double bottomVelocityMPSSetpoint, Rotation2d angle) {
    return new SequentialCommandGroup(
        setShooterVelocitySetpointCommand(
            topVelocityMPSSetpoint, bottomVelocityMPSSetpoint)); // setShooterAngleCommand(angle));
  }

  public Command setShooterVelocitySetpointCommand(
      double topVelocityMPSSetpoint, double bottomVelocityMPSSetpoint) {
    return new InstantCommand(
        () -> {
          runClosedLoop = true;
          topVelocityMPS = topVelocityMPSSetpoint;
          bottomVelocityMPS = bottomVelocityMPSSetpoint;
        },
        this);
  }

  public Command setShooterVoltageSetpointCommand(
      double topVoltsSetpoint, double bottomVoltsSetpoint) {
    return new InstantCommand(
        () -> {
          runClosedLoop = false;
          shooterWheelTop.setFlywheelsVolts(topVoltsSetpoint);
          shooterWheelBottom.setFlywheelsVolts(bottomVoltsSetpoint);
        },
        this);
  }

  // public Command setShooterAngleCommand(Rotation2d angle) {
  //   return new InstantCommand(
  //       () -> {
  //         anglerIO.setGoal(angle);
  //         anglerIO.initPID();
  //       },
  //       this);
  // }

  public Command setIndexerVoltage(double volts) {
    return new InstantCommand(() -> indexerVoltage = volts, this);
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
    // anglerIO.updateInputs(anglerInputsAutoLogged);
    indexerIO.updateInputs(indexerIOInputsAutoLogged);

    Logger.processInputs("Shooter/TopWheel", shooterWheelIOInputsAutoLoggedTop);
    Logger.processInputs("Shooter/BottomWheel", shooterWheelIOInputsAutoLoggedBottom);
    // Logger.processInputs("Shooter/Angler", anglerInputsAutoLogged);
    Logger.processInputs("Shooter/Indexer", indexerIOInputsAutoLogged);

    if (DriverStation.isDisabled()) {
      shooterWheelTop.setFlywheelsVolts(0.0);
      shooterWheelBottom.setFlywheelsVolts(0.0);
      indexerIO.setIndexerVolts(0.0);
      // anglerIO.setAnglerVolts(0.0);
    }

    if (runShooter) {
      if (runClosedLoop) {
        shooterWheelTop.setFlywheelsVelocity(topVelocityMPS);
        shooterWheelBottom.setFlywheelsVelocity(bottomVelocityMPS);
      } else {
        shooterWheelTop.setFlywheelsVolts(topVolts);
        shooterWheelBottom.setFlywheelsVolts(bottomVolts);
      }
    }

    // anglerIO.executePID();

    indexerIO.setIndexerVolts(indexerVoltage);
  }

  public Command getSysIDTests() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(4),
                Units.Seconds.of(15),
                (state) -> sysIDStateLogger(state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  shooterWheelBottom.setFlywheelsVolts(voltage.magnitude());
                  shooterWheelTop.setFlywheelsVolts(voltage.magnitude());
                },
                null, // No log consumer, since external logger
                this));

    return new SequentialCommandGroup(
        startLoggingRoutine(),
        Commands.waitSeconds(3),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(3),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(3),
        stopLoggingRoutine());
    // For rev logs extract using wpilib's data log tool:
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
    // For talon logs extract using phoenix tuner x:
    // https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
  }

  public Command startLoggingRoutine() {
    return Commands.runOnce(() -> SignalLogger.start(), this);
  }

  public void sysIDStateLogger(String state) {
    SignalLogger.writeString("Shooter/sysIDTestState", state);
  }

  public Command stopLoggingRoutine() {
    return Commands.runOnce(() -> SignalLogger.stop(), this);
  }
}
