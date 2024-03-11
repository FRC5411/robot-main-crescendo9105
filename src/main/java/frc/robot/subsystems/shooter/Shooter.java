// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.math.ScrewArmFeedforward;
import java.util.function.DoubleSupplier;
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

  private ShooterVisualizer anglerVisualizer = new ShooterVisualizer(new Rotation2d());

  private Rotation2d anglerSetpoint = null;
  private Double launcherSetpointMPS = null;

  private boolean angleEncoderCalibrated = false;
  private Rotation2d angleOffset = new Rotation2d();
  private Rotation2d currentAngle = new Rotation2d();

  public static enum AnglerSetpoints {
    AIM(() -> TargetingSystem.getLaunchMapAngle()),
    CLIMB(() -> Rotation2d.fromDegrees(23.0)),
    INTAKE(() -> Rotation2d.fromDegrees(40.0));

    private Supplier<Rotation2d> angleSupplier;

    AnglerSetpoints(Supplier<Rotation2d> angle) {
      this.angleSupplier = angle;
    }

    public Rotation2d getAngle() {
      return angleSupplier.get();
    }
  }

  public static enum LauncherSetpoints {
    EJECT(() -> 5.0),
    SPEAKER_SHOT(() -> 38.0),
    FULL_SPEED(() -> 42.0),
    STOP(() -> 0.0);

    private DoubleSupplier speedSupplierMPS;

    LauncherSetpoints(DoubleSupplier speedMPS) {
      this.speedSupplierMPS = speedMPS;
    }

    public double getSpeedMPS() {
      return speedSupplierMPS.getAsDouble();
    }
  }

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
        anglerVisualizer =
            new ShooterVisualizer(anglerIOInputs.anglerRelativePosition.plus(angleOffset));
      }
      currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);

      resetAnglerFeedback();
    }

    currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);

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

    anglerVisualizer.updateShooterAngle(currentAngle);

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
    }
    if (stopLaunchers) {
      launcherSetpointMPS = null;
      launcherIO.setTopVolts(0.0);
      launcherIO.setBottomVolts(0.0);
    }
  }

  /** Reset the angler controller profile */
  public void resetAnglerFeedback() {
    anglerFeedback.reset(currentAngle.getDegrees(), 0.0);
  }

  /** Set the voltage of the angler motor */
  public void setAnglerVolts(double volts) {
    anglerIO.setVolts(volts);
  }

  /** Set the voltage of the launcher motors */
  public void setLauncherVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
    launcherIO.setTopVolts(topFlywheelVolts);
    launcherIO.setBottomVolts(bottomFlywheelVolts);
  }

  /** Set the position setpoint of the angler mechanism */
  public void setAnglerPosition(Rotation2d position) {
    anglerSetpoint = position;

    DoubleSupplier errorDegrees = () -> Math.abs(anglerSetpoint.minus(currentAngle).getDegrees());
    Logger.recordOutput("Shooter/ErrorDegrees", errorDegrees.getAsDouble());

    if (anglerSetpoint != null && errorDegrees.getAsDouble() < 2.0) {
      resetAnglerFeedback();
    }
  }

  /** Set the velocity setpoint of the launcher flywheels */
  public void setLauncherVelocityMPS(Double velocityMPS) {
    launcherSetpointMPS = velocityMPS;
  }

  /** Set all of the motors to a desired state */
  public void setAllMotors(AnglerSetpoints anglerGoal, LauncherSetpoints launcherGoal) {
    setAnglerPosition(anglerGoal.getAngle());
    setLauncherVelocityMPS(launcherGoal.getSpeedMPS());
  }

  /** Set all of the motors to a desired state */
  public void setAllMotors(Rotation2d anglerGoal, double launcherGoal) {
    setAnglerPosition(anglerGoal);
    setLauncherVolts(launcherGoal, launcherGoal);
  }

  /** Returns a command to set the motors to a desired state */
  public Command setShooterState(AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setAnglerPosition(anglerState.getAngle());
          setLauncherVelocityMPS(launcherState.getSpeedMPS());
        },
        (interupted) -> {
          stopMotors(true, true);
        },
        () -> false,
        this);
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

  /** Returns the error of the top motor */
  public double getTopLauncherError() {
    return launcherIOInputs.topFlywheelErrorMPS;
  }

  /** Returns the error of the bottom motor */
  public double getBottomLauncherError() {
    return launcherIOInputs.bottomFlywheelErrorMPS;
  }
}
