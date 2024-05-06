// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    IDLE(() -> Rotation2d.fromDegrees(100.0), () -> 0.0),
    INTAKE(() -> Rotation2d.fromDegrees(-49), () -> -12.0),
    DEBUGGING(() -> Rotation2d.fromDegrees(
      new LoggedTunableNumber("YoshiSetpoint", 0.0).get()), () -> 0.0);

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

  @AutoLogOutput(key = "Yoshivator/CurrentSetpoint")
  private YoshivatorSetpoints currentSetpoint = YoshivatorSetpoints.IDLE;

  private ManipulatorIO manipulatorIO;
  private ManipulatorIOInputsAutoLogged manipulatorIOInputs = new ManipulatorIOInputsAutoLogged();

  private ProfiledPIDController pivotFeedback = new ProfiledPIDController(
    0.0, 0.0, 0.0, 
    new TrapezoidProfile.Constraints(0.0, 0.0));
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.0);

  private LoggedTunableNumber pivotFeedbackP;
  private LoggedTunableNumber pivotFeedbackI;
  private LoggedTunableNumber pivotFeedbackD;
  private LoggedTunableNumber pivotFeedbackV;
  private LoggedTunableNumber pivotFeedbackA;

  private YoshiVisualizer yoshiVisualizer = new YoshiVisualizer(new Rotation2d());

  public Yoshivator(ManipulatorIO manipulatorIO) {
    this.manipulatorIO = manipulatorIO;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          pivotFeedback.setPID(0.1, 0.0, 0.0);
          pivotFeedback.setConstraints(new TrapezoidProfile.Constraints(1600.0, 800.0));
          break;
        case SIM:
          pivotFeedback.setPID(2.0, 0.0, 0.0);
          pivotFeedback.setConstraints(new TrapezoidProfile.Constraints(1000.0, 500.0));
          break;
        default:
          pivotFeedback.setPID(0.0, 0.0, 0.0);
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

    yoshiCommandMap.put(YoshiStates.INTAKE, runYoshi(YoshivatorSetpoints.INTAKE));
    yoshiCommandMap.put(YoshiStates.IDLE, runYoshi(YoshivatorSetpoints.IDLE));

    return yoshiCommandMap;
  }

  @Override
  public void periodic() {
    manipulatorIO.updateInputs(manipulatorIOInputs);
    Logger.processInputs("Yoshivator/Manipulator/Inputs", manipulatorIOInputs);

    if (currentSetpoint != null) {
      Rotation2d goal = Rotation2d.fromDegrees(MathUtil.clamp(
        currentSetpoint.getPivotRotation().get().getDegrees(),
        -44.5, 100));

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
    }

    if (DriverStation.isDisabled()) {
      stopMotors(true, true);
    }

    yoshiVisualizer.updateYoshiAngle(manipulatorIOInputs.pivotPosition);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  public Command runYoshi(YoshivatorSetpoints setpoint) {
    return Commands.runOnce(() -> setYoshiSetpoint(setpoint), this);
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

  // Nulls setpoints into manual control
  public void setManualPivotVolts(double volts) {
    currentSetpoint = null;
    manipulatorIO.setPivotVolts(volts);
  }

  // Nulls setpoints into manual control
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