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
import frc.robot.utils.LoggedTunableNumber;
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
  private PIDController launcherFeedback = new PIDController(0.0, 0.0, 0.0);

  private ArmFeedforward anglerFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward indexerFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  // TODO Remove this cause launcher control is gonna be on Talons
  private SimpleMotorFeedforward launcherFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private LoggedTunableNumber anglerFeedbackP =
      new LoggedTunableNumber("Shooter/Tuning/AnglerP", anglerFeedback.getP());
  private LoggedTunableNumber anglerFeedbackI =
      new LoggedTunableNumber("Shooter/Tuning/AnglerI", anglerFeedback.getI());
  private LoggedTunableNumber anglerFeedbackD =
      new LoggedTunableNumber("Shooter/Tuning/AnglerD", anglerFeedback.getD());
  private LoggedTunableNumber anglerFeedbackA =
      new LoggedTunableNumber(
          "Shooter/Tuning/AnglerAccel", anglerFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber anglerFeedbackV =
      new LoggedTunableNumber(
          "Shooter/Tuning/AnglerVel", anglerFeedback.getConstraints().maxVelocity);

  private LoggedTunableNumber indexerFeedbackP =
      new LoggedTunableNumber("Shooter/Tuning/indexerP", indexerFeedback.getP());
  private LoggedTunableNumber indexerFeedbackI =
      new LoggedTunableNumber("Shooter/Tuning/indexerI", indexerFeedback.getI());
  private LoggedTunableNumber indexerFeedbackD =
      new LoggedTunableNumber("Shooter/Tuning/indexerD", indexerFeedback.getD());

  private LoggedTunableNumber launcherFeedbackP =
      new LoggedTunableNumber("Shooter/Tuning/launcherP", launcherFeedback.getP());
  private LoggedTunableNumber launcherFeedbackI =
      new LoggedTunableNumber("Shooter/Tuning/launcherI", launcherFeedback.getI());
  private LoggedTunableNumber launcherFeedbackD =
      new LoggedTunableNumber("Shooter/Tuning/launcherD", launcherFeedback.getD());

  private Rotation2d anglerSetpoint = null;
  private Double indexerSetpointRPM = null;
  private Double launcherSetpointRPM = null;

  /** Creates a new Shooter. */
  public Shooter(AnglerIO anglerIO, IndexerIO indexerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.indexerIO = indexerIO;
    this.launcherIO = launcherIO;

    anglerFeedback.enableContinuousInput(-Math.PI, Math.PI);

    anglerFeedback.setTolerance(0.001, 0.0);
    indexerFeedback.setTolerance(0.0, 5.0);
    launcherFeedback.setTolerance(0.0, 5.0);
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

    /* Launcher */
    if (launcherFeedbackP.hasChanged(hashCode())
        || launcherFeedbackI.hasChanged(hashCode())
        || launcherFeedbackD.hasChanged(hashCode())) {
      launcherFeedback.setP(launcherFeedbackP.get());
      launcherFeedback.setI(launcherFeedbackI.get());
      launcherFeedback.setD(launcherFeedbackD.get());
    }
  }

  public void setAngler(Rotation2d desiredAngle) {
    anglerSetpoint = desiredAngle;

    Logger.recordOutput("Shooter/AnglerController/Setpoint", anglerSetpoint);
  }

  public void setIndexerVelocity(double desiredVelocityRPM) {
    indexerSetpointRPM = desiredVelocityRPM;

    Logger.recordOutput("Shooter/IndexerController/Setpoint", indexerSetpointRPM);
  }

  public void setLauncher(double desiredVelocityRPM) {
    launcherSetpointRPM = desiredVelocityRPM;

    Logger.recordOutput("Shooter/LauncherController/Setpoint", launcherSetpointRPM);
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
