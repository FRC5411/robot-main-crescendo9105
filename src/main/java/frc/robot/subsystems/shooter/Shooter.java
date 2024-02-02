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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.indexer.IndexerIO;
import frc.robot.subsystems.shooter.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableNumber;

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
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
