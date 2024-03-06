// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.math.ScrewArmFeedforward;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem */
public class Shooter extends SubsystemBase {
  private AnglerIO anglerIO;
  private AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();
  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

  // Default to real values
  private ProfiledPIDController anglerFeedback =
      new ProfiledPIDController(0.49, 2.0, 0.018, new TrapezoidProfile.Constraints(1000.0, 1000.0));
  private ScrewArmFeedforward anglerFeedforward = new ScrewArmFeedforward(0.2, 0.0);

  private LoggedTunableNumber anglerFeedbackP;
  private LoggedTunableNumber anglerFeedbackI;
  private LoggedTunableNumber anglerFeedbackD;
  private LoggedTunableNumber anglerFeedbackV;
  private LoggedTunableNumber anglerFeedbackA;

  private LoggedTunableNumber anglerFeedforwardU;
  private LoggedTunableNumber anglerFeedforwardL;

  private ShooterVisualizer anglerVisualizer = new ShooterVisualizer();

  private Rotation2d anglerSetpoint = null;
  private Double launcherSetpointMPS = null;

  private boolean anglerStopped = false;
  private boolean launcherStopped = false;

  private boolean angleEncoderCalibrated = false;
  private Rotation2d angleOffset = new Rotation2d();
  private Rotation2d currentAngle = new Rotation2d();

  /** Creates a new Shooter. */
  public Shooter(AnglerIO anglerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.launcherIO = launcherIO;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          anglerFeedback.setP(0.49);
          anglerFeedback.setI(2.0);
          anglerFeedback.setD(0.018);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(1000.0, 1000.0));

          anglerFeedforward.updateU(0.2);
          anglerFeedforward.updateL(0.0);
          break;
        case SIM:
          anglerFeedback.setP(0.1);
          anglerFeedback.setI(0.0);
          anglerFeedback.setD(0.5);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(50.0, 50.0));

          anglerFeedforward.updateU(0.0);
          anglerFeedforward.updateL(0.0);
          break;
        default:
          anglerFeedback.setP(0.0);
          anglerFeedback.setI(0.0);
          anglerFeedback.setD(0.0);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          anglerFeedforward.updateU(0.0);
          anglerFeedforward.updateL(0.0);
          break;
      }
    }

    anglerFeedbackP = new LoggedTunableNumber("Shooter/Angler/Feedback/P", anglerFeedback.getP());
    anglerFeedbackI = new LoggedTunableNumber("Shooter/Angler/Feedback/I", anglerFeedback.getI());
    anglerFeedbackD = new LoggedTunableNumber("Shooter/Angler/Feedback/D", anglerFeedback.getD());
    anglerFeedbackV =
        new LoggedTunableNumber(
            "Shooter/Angler/Feedback/V", anglerFeedback.getConstraints().maxVelocity);
    anglerFeedbackA =
        new LoggedTunableNumber(
            "Shooter/Angler/Feedback/A", anglerFeedback.getConstraints().maxAcceleration);
    anglerFeedforwardU =
        new LoggedTunableNumber("Shooter/Angler/Feedforward/U", anglerFeedforward.getU());
    anglerFeedforwardL =
        new LoggedTunableNumber("Shooter/Angler/Feedforward/L", anglerFeedforward.getL());

    resetAnglerFeedback();
    anglerFeedback.setTolerance(0.25);
    anglerFeedback.setIZone(20.0);
    anglerFeedback.setIntegratorRange(-0.5, 0.5);
  }

  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerIOInputs);
    Logger.processInputs("Shooter/Angler/Inputs", anglerIOInputs);
    launcherIO.updateInputs(launcherIOInputs);
    Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors(true, true);
    }

    if (!angleEncoderCalibrated) {
      for (int i = 0; i < 100; i++) {
        if (anglerIOInputs.anglerDutyCycleFrequency == 955) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.anglerAbsolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.anglerRelativePosition);
          angleEncoderCalibrated = true;

          break;
        }
        if (i == 99) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.anglerAbsolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.anglerRelativePosition);
          angleEncoderCalibrated = true;

          break;
        }
      }
      currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);

      resetAnglerFeedback();
    }

    currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);
    Logger.recordOutput("Shooter/Angler/currentPosition", currentAngle);

    if (anglerSetpoint != null) {
      double anglerFeedbackOutput =
          anglerFeedback.calculate(currentAngle.getDegrees(), anglerSetpoint.getDegrees());
      double anglerFeedforwardOutput = anglerFeedforward.calculate(currentAngle, anglerSetpoint);

      double anglerCombinedOutput = (anglerFeedbackOutput + anglerFeedforwardOutput);

      anglerIO.setVolts(anglerCombinedOutput);

      Logger.recordOutput("Shooter/Angler/Feedback/Output", anglerFeedbackOutput);
      Logger.recordOutput("Shooter/Angler/Feedforward/Output", anglerFeedforwardOutput);
      Logger.recordOutput("Shooter/Angler/CombinedOutput", anglerCombinedOutput);
    }

    if (launcherSetpointMPS != null) {
      launcherIO.setTopVelocity(launcherSetpointMPS);
      launcherIO.setBottomVelocity(launcherSetpointMPS);
    }

    anglerVisualizer.updateShooterAngle(anglerIOInputs.anglerRelativePosition);
    Logger.recordOutput("Shooter/Angler/Stopped", anglerStopped);
    Logger.recordOutput("Shooter/Launcher/Stopped", launcherStopped);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  /** Checks if tunable numbers have changed, if so update controllers */
  private void updateTunableNumbers() {
    if (anglerFeedbackP.hasChanged(hashCode())
        || anglerFeedbackI.hasChanged(hashCode())
        || anglerFeedbackD.hasChanged(hashCode())
        || anglerFeedbackV.hasChanged(hashCode())
        || anglerFeedbackA.hasChanged(hashCode())) {
      anglerFeedback.setP(anglerFeedbackP.get());
      anglerFeedback.setI(anglerFeedbackI.get());
      anglerFeedback.setD(anglerFeedbackD.get());

      anglerFeedback.setConstraints(
          new TrapezoidProfile.Constraints(anglerFeedbackV.get(), anglerFeedbackA.get()));
    }
    if (anglerFeedforwardU.hasChanged(hashCode()) || anglerFeedforwardL.hasChanged(hashCode())) {
      anglerFeedforward.updateU(anglerFeedforwardU.get());
      anglerFeedforward.updateL(anglerFeedforwardL.get());
    }
  }

  /** Stop specified motors and set their setpoints to null */
  public void stopMotors(boolean stopAngler, boolean stopLaunchers) {
    if (stopAngler) {
      anglerSetpoint = null;
      anglerIO.setVolts(0.0);

      anglerStopped = true;
    }
    if (stopLaunchers) {
      launcherSetpointMPS = null;
      launcherIO.setTopVolts(0.0);
      launcherIO.setBottomVolts(0.0);

      launcherStopped = true;
    }
  }

  /** Reset the angler controller profile */
  public void resetAnglerFeedback() {
    anglerFeedback.reset(currentAngle.getDegrees(), 0.0);
  }

  /** Set the voltage of the angler motor */
  public void setAnglerVolts(double volts) {
    anglerIO.setVolts(volts);

    anglerStopped = false;
  }

  /** Set the voltage of the launcher motors */
  public void setLauncherVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
    launcherIO.setTopVolts(topFlywheelVolts);
    launcherIO.setBottomVolts(bottomFlywheelVolts);

    launcherStopped = false;
  }

  /** Set the voltage of the launcher motors */
  public void setLauncherVolts(double flywheelVolts) {
    launcherIO.setTopVolts(flywheelVolts);
    launcherIO.setBottomVolts(flywheelVolts);

    launcherStopped = false;
  }

  /** Set the position setpoint of the angler mechanism */
  public void setAnglerPosition(Rotation2d position, boolean reset) {
    anglerSetpoint = position;

    if (anglerSetpoint != null && reset) {
      resetAnglerFeedback();
    }

    anglerStopped = false;
  }

  /** Set the velocity setpoint of the launcher flywheels */
  public void setLauncherVelocityMPS(Double velocityMPS) {
    launcherSetpointMPS = velocityMPS;

    launcherStopped = false;
  }

  /** Set all of the motors to a desired state */
  public void setAllMotors(
      Rotation2d anglerPosition, double launcherVelocityMPS, boolean resetProfile) {
    setAnglerPosition(anglerPosition, resetProfile);
    setLauncherVelocityMPS(launcherVelocityMPS);

    anglerStopped = false;
    launcherStopped = false;
  }

  /** Set all of the motors to a desired state */
  public void setAllMotors(
      Supplier<Rotation2d> anglerPosition, double launcherVelocityMPS, boolean resetProfile) {
    setAnglerPosition(anglerPosition.get(), resetProfile);
    setLauncherVelocityMPS(launcherVelocityMPS);

    anglerStopped = false;
    launcherStopped = false;
  }

  /** Returns the angle of the pivot */
  @AutoLogOutput(key = "Shooter/Angler/Position")
  public Rotation2d getAnglerPosition() {
    return currentAngle;
  }

  /** Returns the setpoint state position of the angler feedback */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/SetpointPosition")
  public Rotation2d getAnglerSetpointPosition() {
    return Rotation2d.fromDegrees(anglerFeedback.getSetpoint().position);
  }

  /** Returns the setpoint state velocity of the angler feedback */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/SetpointVelocity")
  public double getAnglerSetpointVelocity() {
    return anglerFeedback.getSetpoint().velocity;
  }

  /** Returns the goal state position of the angler feedback */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/GoalPosition")
  public double getAnglerGoalPosition() {
    return anglerFeedback.getGoal().position;
  }

  /** Returns the position error of the angler feedback */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/PositionError")
  public double getAnglerPositionError() {
    return anglerFeedback.getPositionError();
  }

  /** Returns the velocity error of the angler feedback */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/VelocityError")
  public double getAnglerVelocityError() {
    return anglerFeedback.getVelocityError();
  }

  /** Returns whether or not the Angler is at the goal */
  @AutoLogOutput(key = "Shooter/Angler/Feedback/AtGoal")
  public boolean isAnglerAtGoal() {
    return anglerFeedback.atGoal();
  }

  /** Returns the velocity of the top launcher flywheel */
  @AutoLogOutput(key = "Shooter/TopLauncher/VelocityMPS")
  public double getTopLauncherVelocityMPS() {
    return launcherIOInputs.topFlywheelVelocityMPS;
  }

  /** Returns the velocity of the bottom launcher flyhweel */
  @AutoLogOutput(key = "Shooter/BottomLauncher/VelocityMPS")
  public double getBottomLauncherVelocityMPS() {
    return launcherIOInputs.bottomFlywheelVelocityMPS;
  }

  /** Returns the error of the top motor */
  public double getTopLauncherError() {
    return launcherIOInputs.topFlywheelErrorMPS;
  }

  /** Returns the error of the bottom motor */
  public double getBottomLauncherError() {
    return launcherIOInputs.bottomFlywheelErrorMPS;
  }
}
