// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIO;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Yoshivator extends SubsystemBase {
  public static enum YoshiSetpoints {
    IDLE(Rotation2d.fromDegrees(100), 0),
    GROUND(Rotation2d.fromDegrees(-35), 12),
    AMP_IDLE(Rotation2d.fromDegrees(85), 0),
    AMP_SCORE(Rotation2d.fromDegrees(85), -12);

    private Rotation2d pivotSetpointRotation;
    private double rollerSetpointVolts;

    YoshiSetpoints(Rotation2d pivotSetpointRotation, double rollerSetpointVolts) {
      this.pivotSetpointRotation = pivotSetpointRotation;
      this.rollerSetpointVolts = rollerSetpointVolts;
    }

    public Rotation2d getPivotRotation() {
      return this.pivotSetpointRotation;
    }

    public double getRollerVolts() {
      return this.rollerSetpointVolts;
    }
  }

  private ManipulatorIO manipulatorIO;
  private ManipulatorIOInputsAutoLogged manipulatorIOInputs = new ManipulatorIOInputsAutoLogged();

  private ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(0.2, 0.0, 0.0, new TrapezoidProfile.Constraints(1600.0, 800.0));
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.0);

  private LoggedTunableNumber pivotFeedbackP;
  private LoggedTunableNumber pivotFeedbackI;
  private LoggedTunableNumber pivotFeedbackD;
  private LoggedTunableNumber pivotFeedbackV;
  private LoggedTunableNumber pivotFeedbackA;

  private YoshiVisualizer yoshiVisualizer = new YoshiVisualizer(new Rotation2d());

  @AutoLogOutput(key = "Yoshivator/CurrentCommand")
  private Command currentCommand = null;

  @AutoLogOutput(key = "Yoshivator/CurrentSetpoint")
  private YoshiSetpoints currentSetpoint = null;

  public Yoshivator(ManipulatorIO manipulatorIO) {
    this.manipulatorIO = manipulatorIO;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          pivotFeedback.setP(0.2);
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

  public Command runYoshi(YoshiSetpoints setpoint) {
    currentCommand = Commands.runOnce(() -> setYoshiSetpoint(setpoint), this);
    return currentCommand;
  }

  public Command scoreAmp() {
    currentCommand =
        new FunctionalCommand(
            () -> setYoshiSetpoint(YoshiSetpoints.AMP_IDLE),
            () -> {},
            (interrupted) -> {
              if (!interrupted) {
                setYoshiSetpoint(YoshiSetpoints.AMP_SCORE);
              }
            },
            () ->
                Math.abs(
                        currentSetpoint
                            .getPivotRotation()
                            .minus(manipulatorIOInputs.pivotPosition)
                            .getDegrees())
                    < 5);

    return currentCommand;
  }

  @Override
  public void periodic() {
    manipulatorIO.updateInputs(manipulatorIOInputs);
    Logger.processInputs("Yoshivator/Manipulator/Inputs", manipulatorIOInputs);

    if (currentSetpoint != null) {
      double pivotFeedbackOutput =
          pivotFeedback.calculate(
              manipulatorIOInputs.pivotPosition.getDegrees(),
              currentSetpoint.getPivotRotation().getDegrees());
      double pivotFeedforwardOutput =
          pivotFeedforward.calculate(
              Math.toRadians(pivotFeedback.getSetpoint().position),
              pivotFeedback.getSetpoint().velocity);

      double pivotCombinedOutput = pivotFeedbackOutput + pivotFeedforwardOutput;

      manipulatorIO.setPivotVolts(pivotCombinedOutput);

      manipulatorIO.setRollerVolts(currentSetpoint.getRollerVolts());

      Logger.recordOutput("Yoshivator/Pivot/Feedback/Output", pivotFeedbackOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedforward/Output", pivotFeedforwardOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedback/CombinedOutput", pivotCombinedOutput);
    }

    yoshiVisualizer.updateYoshiAngle(manipulatorIOInputs.pivotPosition);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
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
    currentSetpoint = null;
    if (stopPivot) {
      setPivotVolts(0.0);
    }
    if (stopFlywheel) {
      setRollerVolts(0.0);
    }
  }

  public void setYoshiSetpoint(YoshiSetpoints setpoint) {
    setYoshiSetpoint(setpoint, true);
  }

  public void setYoshiSetpoint(YoshiSetpoints setpoint, boolean shouldReset) {
    currentSetpoint = setpoint;
    if (setpoint != null) {
      pivotFeedback.setGoal(setpoint.getPivotRotation().getDegrees());
      if(shouldReset)
        pivotFeedback.reset(manipulatorIOInputs.pivotPosition.getDegrees());
      setRollerVolts(setpoint.getRollerVolts());
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
