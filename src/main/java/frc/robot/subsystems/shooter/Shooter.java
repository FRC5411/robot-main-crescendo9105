// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.indexer.IndexerIO;
import frc.robot.subsystems.shooter.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private AnglerIO anglerIO;
  private IndexerIO indexerIO;
  private LauncherIO launcherIO;

  private AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();
  private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

  private ProfiledPIDController anglerFeedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private PIDController indexerFeedback = new PIDController(0.0, 0.0, 0.0);

  private ArmFeedforward anglerFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward indexerFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private LoggedTunableNumber anglerFeedbackP =
      new LoggedTunableNumber("Shooter/Angler/Feedback/P", anglerFeedback.getP());
  private LoggedTunableNumber anglerFeedbackI =
      new LoggedTunableNumber("Shooter/Angler/Feedback/I", anglerFeedback.getI());
  private LoggedTunableNumber anglerFeedbackD =
      new LoggedTunableNumber("Shooter/Angler/Feedback/D", anglerFeedback.getD());
  private LoggedTunableNumber anglerFeedbackA =
      new LoggedTunableNumber(
          "Shooter/Angler/Feedback/Accel", anglerFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber anglerFeedbackV =
      new LoggedTunableNumber(
          "Shooter/Angler/Feedback/Vel", anglerFeedback.getConstraints().maxVelocity);

  private LoggedTunableNumber indexerFeedbackP =
      new LoggedTunableNumber("Shooter/Indexer/Feedback/P", indexerFeedback.getP());
  private LoggedTunableNumber indexerFeedbackI =
      new LoggedTunableNumber("Shooter/Indexer/Feedback/I", indexerFeedback.getI());
  private LoggedTunableNumber indexerFeedbackD =
      new LoggedTunableNumber("Shooter/Indexer/Feedback/D", indexerFeedback.getD());

  private Rotation2d anglerSetpoint = null;
  private Double indexerSetpointRPM = null;
  private Double launcherSetpointMPS = null;

  /** Creates a new Shooter. */
  public Shooter(AnglerIO anglerIO, IndexerIO indexerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.indexerIO = indexerIO;
    this.launcherIO = launcherIO;

    anglerFeedback.enableContinuousInput(-Math.PI, Math.PI);

    anglerFeedback.setTolerance(0.001, 0.0);
    indexerFeedback.setTolerance(0.0, 5.0);
  }

  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerIOInputs);
    Logger.processInputs("Shooter/Angler/Inputs", anglerIOInputs);
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Shooter/Indexer/Inputs", indexerIOInputs);
    launcherIO.updateInputs(launcherIOInputs);
    Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors(true, true, true);
    }

    if (anglerSetpoint != null) {
      var anglerFeedbackOutput =
          anglerFeedback.calculate(
                  anglerIOInputs.dutyCycleEncoderRadians, anglerSetpoint.getRadians())
              / 12.0;
      // var anglerFeedforwardOutput =
      //     anglerFeedforward.calculate(anglerIOInputs.angleRadians, anglerIOInputs.velocityRPS)
      //         / 12.0;

      anglerIO.setVolts(anglerFeedbackOutput);

      Logger.recordOutput("Shooter/Angler/Feedback/Output", anglerFeedbackOutput);
      // Logger.recordOutput("Shooter/Angler/Feedforward/Output", anglerFeedforwardOutput);
    }
    if (indexerSetpointRPM != null) {
      var indexerFeedbackOutput =
          indexerFeedback.calculate(indexerIOInputs.velocityRPM, indexerSetpointRPM) / 12.0;
      // var indexerFeedforwardOutput =
      //     indexerFeedforward.calculate(indexerIOInputs.velocityRPM) / 12.0;

      indexerIO.setVolts((indexerFeedbackOutput));

      Logger.recordOutput("Shooter/Indexer/Feedback/Output", indexerFeedbackOutput);
      // Logger.recordOutput("Shooter/Indexer/Feedforward/Output", indexerFeedforwardOutput);
    }
    if (launcherSetpointMPS != null) {
      launcherIO.setVelocity(launcherSetpointMPS);
    }

    updateTunableNumbers();
  }

  /** Update tunable numbers if they've changed */
  private void updateTunableNumbers() {
    // hashCode() updates when class is changed (I think)

    /* Angler */
    if (anglerFeedbackP.hasChanged(hashCode())
        || anglerFeedbackI.hasChanged(hashCode())
        || anglerFeedbackD.hasChanged(hashCode())) {
      anglerFeedback.setP(anglerFeedbackP.get());
      anglerFeedback.setI(anglerFeedbackI.get());
      anglerFeedback.setD(anglerFeedbackD.get());
    }
    if (anglerFeedbackA.hasChanged(hashCode()) || anglerFeedbackV.hasChanged(hashCode())) {
      var newConstraints =
          new TrapezoidProfile.Constraints(anglerFeedbackA.get(), anglerFeedbackV.get());

      anglerFeedback.setConstraints(newConstraints);
    }

    /* Indexer */
    if (indexerFeedbackP.hasChanged(hashCode())
        || indexerFeedbackI.hasChanged(hashCode())
        || indexerFeedbackD.hasChanged(hashCode())) {
      indexerFeedback.setP(indexerFeedbackP.get());
      indexerFeedback.setI(indexerFeedbackI.get());
      indexerFeedback.setD(indexerFeedbackD.get());
    }
  }

  /** Set the volts for the launcher */
  public void setManualAngler(double volts) {
    anglerIO.setVolts(volts);
  }

  /** Set the volts for the launcher */
  public void setManualIndexer(double volts) {
    indexerIO.setVolts(volts);
  }

  /** Set the volts for the launcher */
  public void setManualLauncher(double volts) {
    launcherIO.setVolts(volts, volts);
  }

  /** Set the closed-loop control goal of the angler */
  public void setAngler(Rotation2d desiredAngle) {
    anglerSetpoint = desiredAngle;
  }

  /** Set the closed-loop control goal of the indexer */
  public void setIndexerVelocity(double desiredVelocityRPM) {
    indexerSetpointRPM = desiredVelocityRPM;
  }

  /** Set the closed-loop control goal of the launcher */
  public void setLauncherVelocity(double desiredVelocityMPS) {
    launcherSetpointMPS = desiredVelocityMPS;
  }

  /** Returns the angular position setpoint for the angler */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/Setpoint")
  public Rotation2d getAnglerSetpoint() {
    if (anglerSetpoint != null) {
      return anglerSetpoint;
    } else {
      return new Rotation2d();
    }
  }

  /** Returns the indexer's velocity setpoint in RPM */
  @AutoLogOutput(key = "Shooter/Indexer/Feedback/Setpoint")
  public double getIndexerSetpoint() {
    if (indexerSetpointRPM != null) {
      return indexerSetpointRPM;
    } else {
      return 0.0;
    }
  }

  /** Returns the launcher's velocity setpoint in MPS */
  @AutoLogOutput(key = "Shooter/Launcher/Feedback/Setpoint")
  public double getLauncherVelocity() {
    if (launcherSetpointMPS != null) {
      return launcherSetpointMPS;
    } else {
      return 0.0;
    }
  }

  /** Stop the motors of the shooter subsystem */
  public void stopMotors(boolean stopAngler, boolean stopIndexer, boolean stopLauncher) {
    if (stopAngler) {
      anglerIO.setVolts(0.0);
    }
    if (stopIndexer) {
      indexerIO.setVolts(0.0);
    }
    if (stopLauncher) {
      launcherIO.setVolts(0.0, 0.0);
    }
  }
}
