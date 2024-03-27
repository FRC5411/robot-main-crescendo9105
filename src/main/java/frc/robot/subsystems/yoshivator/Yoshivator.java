// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.RobotStates.YoshiStates;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIO;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;

import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Yoshivator extends SubsystemBase {
  public static enum YoshivatorSetpoints {
    IDLE(() -> Rotation2d.fromDegrees(90.0), () -> 0.0),
    GROUND_AMP(() -> Rotation2d.fromDegrees(-49.0), () -> 12.0),
    GROUND_INTAKE(() -> Rotation2d.fromDegrees(-48.0), () -> -12.0),
    AMP_IDLE(() -> Rotation2d.fromDegrees(78.5), () -> 0.0),
    AMP_SCORE(() -> Rotation2d.fromDegrees(78.5), () -> -5.0),
    HOLD(() -> Yoshivator.pivotPosition, () -> 0.0),
    DEBUGGING(() -> Rotation2d.fromDegrees(new LoggedTunableNumber("e", 0.0).get()), () -> 0.0);

    private Supplier<Rotation2d> pivotSetpointRotation;
    private Supplier<Double> rollerSetpointVolts;

    YoshivatorSetpoints(
        Supplier<Rotation2d> pivotSetpointRotation, Supplier<Double> rollerSetpointVolts) {
      this.pivotSetpointRotation = pivotSetpointRotation;
      this.rollerSetpointVolts = rollerSetpointVolts;
    }

    public Supplier<Rotation2d> getPivotRotation() {
      return this.pivotSetpointRotation;
    }

    public Supplier<Double> getRollerVolts() {
      return this.rollerSetpointVolts;
    }
  }

  public static Rotation2d pivotPosition = null;

  @AutoLogOutput(key = "Yoshivator/CurrentSetpoint")
  private YoshivatorSetpoints currentSetpoint = YoshivatorSetpoints.IDLE;

  private ManipulatorIO manipulatorIO;
  private ManipulatorIOInputsAutoLogged manipulatorIOInputs = new ManipulatorIOInputsAutoLogged();

  private ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(0.02, 0.0, 0.0, new TrapezoidProfile.Constraints(200.0, 100.0));
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.0);

  private LoggedTunableNumber pivotFeedbackP;
  private LoggedTunableNumber pivotFeedbackI;
  private LoggedTunableNumber pivotFeedbackD;
  private LoggedTunableNumber pivotFeedbackV;
  private LoggedTunableNumber pivotFeedbackA;

  private YoshiVisualizer yoshiVisualizer = new YoshiVisualizer(new Rotation2d());

  private LinearFilter yoshiLinearFilter = LinearFilter.movingAverage(20);

  public Yoshivator(ManipulatorIO manipulatorIO) {
    this.manipulatorIO = manipulatorIO;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          pivotFeedback.setP(0.1);
          pivotFeedback.setI(0.0);
          pivotFeedback.setD(0.0);
          pivotFeedback.setConstraints(new TrapezoidProfile.Constraints(1600.0, 800.0));
          break;
        case SIM:
          pivotFeedback.setP(2.0);
          pivotFeedback.setI(0.0);
          pivotFeedback.setD(0.0);
          pivotFeedback.setConstraints(new TrapezoidProfile.Constraints(1000.0, 500.0));
          break;
        default:
          pivotFeedback.setP(0.0);
          pivotFeedback.setI(0.0);
          pivotFeedback.setD(0.0);
          pivotFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));
          break;
      }
    }

    pivotFeedback.setTolerance(1.0);

    pivotFeedbackP = new LoggedTunableNumber("Yoshivator/Pivot/Feedback/P", pivotFeedback.getP());
    pivotFeedbackI = new LoggedTunableNumber("Yoshivator/Pivot/Feedback/I", pivotFeedback.getI());
    pivotFeedbackD = new LoggedTunableNumber("Yoshivator/Pivot/Feedback/D", pivotFeedback.getD());
    pivotFeedbackV =
        new LoggedTunableNumber(
            "Yoshivator/Pivot/Feedback/V", pivotFeedback.getConstraints().maxVelocity);
    pivotFeedbackA =
        new LoggedTunableNumber(
            "Yoshivator/Pivot/Feedback/A", pivotFeedback.getConstraints().maxAcceleration);
  }

  public HashMap<YoshiStates, Command> mapToCommand(YoshiStates desiredState) {
    HashMap<YoshiStates, Command> yoshiCommandMap = new HashMap<>();

      yoshiCommandMap.put(YoshiStates.OFF, offYoshi());
      yoshiCommandMap.put(YoshiStates.GROUND_AMP, runYoshi(YoshivatorSetpoints.GROUND_AMP));
      yoshiCommandMap.put(YoshiStates.GROUND_INTAKE, runYoshi(YoshivatorSetpoints.GROUND_INTAKE));
      yoshiCommandMap.put(YoshiStates.AMP, scoreAmp());
      yoshiCommandMap.put(YoshiStates.IDLE, runYoshi(YoshivatorSetpoints.IDLE));

      return yoshiCommandMap;
  }

  @Override
  public void periodic() {
    manipulatorIO.updateInputs(manipulatorIOInputs);
    Logger.processInputs("Yoshivator/Manipulator/Inputs", manipulatorIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors(true, true);
    }

    if (yoshiLinearFilter.calculate(manipulatorIOInputs.rollerAppliedCurrentAmps[0]) > 60.0
        && (currentSetpoint == YoshivatorSetpoints.GROUND_INTAKE
            || currentSetpoint == YoshivatorSetpoints.GROUND_AMP)) {
      stopMotors(false, true);
      Logger.recordOutput("Limited", true);
    } else {
      Logger.recordOutput("Limited", false);
    }

    if (currentSetpoint != null) {
      Rotation2d goal = Rotation2d.fromDegrees(MathUtil.clamp(
        currentSetpoint.getPivotRotation().get().getDegrees(),
        -30, 90));

      double pivotFeedbackOutput =
          pivotFeedback.calculate(
              manipulatorIOInputs.pivotPosition.getDegrees(),
              goal.getDegrees());
      double pivotFeedforwardOutput =
          pivotFeedforward.calculate(
              Math.toRadians(pivotFeedback.getSetpoint().position),
              pivotFeedback.getSetpoint().velocity);

      double pivotCombinedOutput = (pivotFeedbackOutput + pivotFeedforwardOutput) * -1.0;

      setPivotVolts(pivotCombinedOutput);

      setRollerVolts(currentSetpoint.getRollerVolts().get());

      Logger.recordOutput("Yoshivator/Pivot/Feedback/Output", pivotFeedbackOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedforward/Output", pivotFeedforwardOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedback/CombinedOutput", pivotCombinedOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedback/PositionError", pivotFeedback.getPositionError());
    }

    pivotPosition = manipulatorIOInputs.pivotPosition;
    yoshiVisualizer.updateYoshiAngle(manipulatorIOInputs.pivotPosition);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  public Command runYoshi(YoshivatorSetpoints setpoint) {
    return Commands.runOnce(() -> setYoshiSetpoint(setpoint), this);
  }

  public Command scoreAmp() {
    return new FunctionalCommand(
        () -> setYoshiSetpoint(YoshivatorSetpoints.AMP_IDLE),
        () -> {},
        (interrupted) -> {
          setYoshiSetpoint(YoshivatorSetpoints.AMP_SCORE, false);
        },
        () ->
            Math.abs(
                    currentSetpoint
                        .getPivotRotation()
                        .get()
                        .minus(manipulatorIOInputs.pivotPosition)
                        .getDegrees())
                < 10.0,
        this);
  }

  public Command offYoshi() {
    return Commands.runOnce(
        () -> {
          setYoshiSetpoint(null);
          stopMotors(true, false);
        },
        this);
  }

  private void updateTunableNumbers() {
    if (pivotFeedbackP.hasChanged(hashCode())
        || pivotFeedbackI.hasChanged(hashCode())
        || pivotFeedbackD.hasChanged(hashCode())
        || pivotFeedbackV.hasChanged(hashCode())
        || pivotFeedbackA.hasChanged(hashCode())) {
      pivotFeedback.setP(pivotFeedbackP.get());
      pivotFeedback.setI(pivotFeedbackI.get());
      pivotFeedback.setD(pivotFeedbackD.get());

      pivotFeedback.setConstraints(
          new TrapezoidProfile.Constraints(pivotFeedbackV.get(), pivotFeedbackA.get()));
    }
  }

  public void stopMotors(boolean stopPivot, boolean stopFlywheel) {
    if (stopPivot) {
      setPivotVolts(0.0);
    }
    if (stopFlywheel) {
      setRollerVolts(0.0);
    }
  }

  public void setYoshiSetpoint(YoshivatorSetpoints setpoint) {
    setYoshiSetpoint(setpoint, true);
  }

  public void setYoshiSetpoint(YoshivatorSetpoints setpoint, boolean shouldReset) {
    currentSetpoint = setpoint;
    if (setpoint != null) {
      pivotFeedback.setGoal(setpoint.getPivotRotation().get().getDegrees());
      if (shouldReset) pivotFeedback.reset(manipulatorIOInputs.pivotPosition.getDegrees());
      setRollerVolts(setpoint.getRollerVolts().get());
    }
  }

  public void setPivotVolts(double volts) {
    manipulatorIO.setPivotVolts(volts);
  }

  public void setRollerVolts(double volts) {
    manipulatorIO.setRollerVolts(volts);
  }

  public void setManualPivotVolts(double volts) {
    currentSetpoint = null;
    manipulatorIO.setPivotVolts(volts);
  }

  public void setManualRollerVolts(double volts) {
    currentSetpoint = null;
    manipulatorIO.setRollerVolts(volts);
  }

  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/PositionGoal")
  public double getPivotPositionGoal() {
    return Math.toRadians(pivotFeedback.getGoal().position);
  }

  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/PositionSetpoint")
  public double getPivotPositionSetpoint() {
    return Math.toRadians(pivotFeedback.getSetpoint().position);
  }

  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/AtGoal")
  public boolean isPivotAtGoal() {
    return pivotFeedback.atGoal();
  }

  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/AtGoal")
  public boolean isPivotAtSetpoint() {
    return pivotFeedback.atSetpoint();
  }
}