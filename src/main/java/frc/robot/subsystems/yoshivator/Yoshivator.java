// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIO;
import frc.robot.subsystems.yoshivator.manipulator.ManipulatorIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Yoshivator subsystem */
public class Yoshivator extends SubsystemBase {
  private ManipulatorIO manipulatorIO;
  private ManipulatorIOInputsAutoLogged manipulatorIOInputs = new ManipulatorIOInputsAutoLogged();

  private ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  private LoggedTunableNumber pivotFeedbackP =
      new LoggedTunableNumber("Yoshivator/Pivot/Feedback/P", pivotFeedback.getP());
  private LoggedTunableNumber pivotFeedbackI =
      new LoggedTunableNumber("Yoshivator/Pivot/Feedback/I", pivotFeedback.getI());
  private LoggedTunableNumber pivotFeedbackD =
      new LoggedTunableNumber("Yoshivator/Pivot/Feedback/D", pivotFeedback.getD());
  private LoggedTunableNumber pivotFeedbackV =
      new LoggedTunableNumber(
          "Yoshivator/Pivot/Feedback/V", pivotFeedback.getConstraints().maxVelocity);
  private LoggedTunableNumber pivotFeedbackA =
      new LoggedTunableNumber(
          "Yoshivator/Pivot/Feedback/A", pivotFeedback.getConstraints().maxAcceleration);

  private Rotation2d pivotSetpoint = null;

  /** Creates a new Yoshivator. */
  public Yoshivator(ManipulatorIO manipulatorIO) {
    this.manipulatorIO = manipulatorIO;
  }

  @Override
  public void periodic() {
    manipulatorIO.updateInputs(manipulatorIOInputs);
    Logger.processInputs("Yoshivator/Manipulator/Inputs", manipulatorIOInputs);

    if (pivotSetpoint != null) {
      double pivotFeedbackOutput =
          pivotFeedback.calculate(
              manipulatorIOInputs.pivotPosition.getDegrees(), pivotSetpoint.getDegrees());
      double pivotFeedforwardOutput =
          pivotFeedforward.calculate(
              pivotFeedback.getSetpoint().position, pivotFeedback.getSetpoint().velocity);

      double pivotCombinedOutput = pivotFeedbackOutput + pivotFeedforwardOutput;

      manipulatorIO.setPivotVolts(pivotCombinedOutput);

      Logger.recordOutput("Yoshivator/Pivot/Feedback/Output", pivotFeedbackOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedforward/Output", pivotFeedforwardOutput);
      Logger.recordOutput("Yoshivator/Pivot/Feedback/Output", pivotCombinedOutput);
    }

    updateTunableNumbers();
  }

  /** Checks if tunable numbers have changed, if so update controllers */
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

  /** Stops the motors of the Yoshi */
  public void stopMotors(boolean stopPivot, boolean stopFlywheel) {
    if (stopPivot) {
      pivotSetpoint = null;
      setPivotVolts(0.0);
    }
    if (stopFlywheel) {
      setFlywheelVolts(0.0);
    }
  }

  /** Set the goal of the pviot controller */
  public void setPivotSetpoint(Rotation2d desiredPosition) {
    pivotSetpoint = desiredPosition;
  }

  /** Set the voltage of the pivot motor */
  public void setPivotVolts(double volts) {
    manipulatorIO.setPivotVolts(volts);
  }

  /** Set the voltage of the flywheel motor */
  public void setFlywheelVolts(double volts) {
    manipulatorIO.setFlywheelVolts(volts);
  }

  /** Returns the pivot's position goal */
  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/PositionGoal")
  public double getPivotPositionGoal() {
    return pivotFeedback.getGoal().position;
  }

  /** Returns the pivot's velocity goal */
  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/VelocityGoal")
  public double getPivotVelocityGoal() {
    return pivotFeedback.getGoal().velocity;
  }

  /** Returns if the pivot is at the goal or not */
  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/AtGoal")
  public boolean isPivotAtGoal() {
    return pivotFeedback.atGoal();
  }

  /** Returns the pivot's position setpoint */
  @AutoLogOutput(key = "Yoshivator/Pivot/Feedback/PositionSetpoint")
  public double getPivotPositionSetpoint() {
    return pivotFeedback.getSetpoint().position;
  }
}
