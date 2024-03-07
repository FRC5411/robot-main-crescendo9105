// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.climb.ClimbVisualizer.ClimbSide;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Climb subsystem */
public class Climb extends SubsystemBase {
  private ClimbIO climbIO;
  private ClimbIOInputsAutoLogged climbIOInputs = new ClimbIOInputsAutoLogged();

  private ProfiledPIDController leftClimbFeedback =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(100.0, 100.0));
  private ProfiledPIDController rightClimbFeedback =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(100.0, 100.0));

  private ArmFeedforward leftClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private ArmFeedforward rightClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  private LoggedTunableNumber leftFeedbackP;
  private LoggedTunableNumber leftFeedbackI;
  private LoggedTunableNumber leftFeedbackD;
  private LoggedTunableNumber leftFeedbackA;
  private LoggedTunableNumber leftFeedbackV;

  private LoggedTunableNumber rightFeedbackP;
  private LoggedTunableNumber rightFeedbackI;
  private LoggedTunableNumber rightFeedbackD;
  private LoggedTunableNumber rightFeedbackA;
  private LoggedTunableNumber rightFeedbackV;

  private Rotation2d leftAngleSetpoint = null;
  private Rotation2d rightAngleSetpoint = null;

  private ClimbVisualizer leftVisualizer = new ClimbVisualizer(ClimbSide.LEFT);
  private ClimbVisualizer rightVisualizer = new ClimbVisualizer(ClimbSide.RIGHT);

  /** Creates a new Climb. */
  public Climb(ClimbIO io) {
    this.climbIO = io;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          leftClimbFeedback.setP(1.0);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          rightClimbFeedback.setP(1.0);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          break;
        case SIM:
          leftClimbFeedback.setP(1.0);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          rightClimbFeedback.setP(1.0);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          break;
        default:
          leftClimbFeedback.setP(0.0);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          rightClimbFeedback.setP(0.0);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          break;
      }
    }

    leftFeedbackP = new LoggedTunableNumber("Climb/LeftArm/Feedback/P", leftClimbFeedback.getP());
    leftFeedbackI = new LoggedTunableNumber("Climb/LeftArm/Feedback/I", leftClimbFeedback.getI());
    leftFeedbackD = new LoggedTunableNumber("Climb/LeftArm/Feedback/D", leftClimbFeedback.getD());
    leftFeedbackA =
        new LoggedTunableNumber(
            "Climb/LeftArm/Feedback/V", leftClimbFeedback.getConstraints().maxVelocity);
    leftFeedbackV =
        new LoggedTunableNumber(
            "Climb/LeftArm/Feedback/A", leftClimbFeedback.getConstraints().maxAcceleration);

    rightFeedbackP =
        new LoggedTunableNumber("Climb/RightArm/Feedback/P", rightClimbFeedback.getP());
    rightFeedbackI =
        new LoggedTunableNumber("Climb/RightArm/Feedback/I", rightClimbFeedback.getI());
    rightFeedbackD =
        new LoggedTunableNumber("Climb/RightArm/Feedback/D", rightClimbFeedback.getD());
    rightFeedbackA =
        new LoggedTunableNumber(
            "Climb/RightArm/Feedback/V", rightClimbFeedback.getConstraints().maxVelocity);
    rightFeedbackV =
        new LoggedTunableNumber(
            "Climb/RightArm/Feedback/A", rightClimbFeedback.getConstraints().maxAcceleration);

    leftClimbFeedback.setTolerance(0.8);
    rightClimbFeedback.setTolerance(0.8);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOInputs);
    Logger.processInputs("Climb/Inputs", climbIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (leftAngleSetpoint != null) {
      var leftFeedbackOutput =
          leftClimbFeedback.calculate(
              climbIOInputs.leftPosition.getDegrees(), leftAngleSetpoint.getDegrees());
      // TODO Need to add angle offset
      var leftFeedforwardOutput =
          leftClimbFeedforward.calculate(
              climbIOInputs.leftPosition.getRadians(), leftClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/LeftArm/Feedback/Output", leftFeedbackOutput);
      Logger.recordOutput("Climb/LeftArm/Feedforward/Output", leftFeedforwardOutput);

      climbIO.setLeftVolts(leftFeedbackOutput + leftFeedforwardOutput);
    }

    if (rightAngleSetpoint != null) {
      var rightFeedbackOutput =
          rightClimbFeedback.calculate(
              climbIOInputs.rightPosition.getDegrees(), rightAngleSetpoint.getDegrees());
      // TODO Need to add angle offset
      var rightFeedforwardOutput =
          rightClimbFeedforward.calculate(
              climbIOInputs.rightPosition.getRadians(), rightClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/RightArm/Feedback/Output", rightFeedbackOutput);
      Logger.recordOutput("Climb/RightArm/Feedforward/Output", rightFeedforwardOutput);

      climbIO.setRightVolts(rightFeedbackOutput + rightFeedforwardOutput);
    }

    leftVisualizer.updateClimbAngle(climbIOInputs.leftPosition);
    rightVisualizer.updateClimbAngle(climbIOInputs.rightPosition);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  /** Checks if tunable numbers have changed, if so update controllers */
  private void updateTunableNumbers() {
    // hashCode() updates when class is changed (I think)
    if (leftFeedbackP.hasChanged(hashCode())
        || leftFeedbackI.hasChanged(hashCode())
        || leftFeedbackD.hasChanged(hashCode())) {
      leftClimbFeedback.setP(leftFeedbackP.get());
      leftClimbFeedback.setI(leftFeedbackI.get());
      leftClimbFeedback.setD(leftFeedbackD.get());
    }
    if (leftFeedbackA.hasChanged(hashCode()) || leftFeedbackV.hasChanged(hashCode())) {
      var newConstraints =
          new TrapezoidProfile.Constraints(leftFeedbackA.get(), leftFeedbackV.get());

      leftClimbFeedback.setConstraints(newConstraints);
    }

    if (rightFeedbackP.hasChanged(hashCode())
        || rightFeedbackI.hasChanged(hashCode())
        || rightFeedbackD.hasChanged(hashCode())) {
      rightClimbFeedback.setP(rightFeedbackP.get());
      rightClimbFeedback.setI(rightFeedbackI.get());
      rightClimbFeedback.setD(rightFeedbackD.get());
    }
    if (rightFeedbackA.hasChanged(hashCode()) || rightFeedbackV.hasChanged(hashCode())) {
      var newConstraints =
          new TrapezoidProfile.Constraints(rightFeedbackA.get(), rightFeedbackV.get());

      rightClimbFeedback.setConstraints(newConstraints);
    }
  }

  /** Set the volts of each arm */
  public void setVolts(double leftVolts, double rightVolts) {
    climbIO.setLeftVolts(leftVolts);
    climbIO.setRightVolts(rightVolts);
  }

  /** Sets the volts of the left climb */
  public void setVoltsLeft(double volts) {
    climbIO.setLeftVolts(volts);
  }

  /** Sets the volts of the right climb */
  public void setVoltsRight(double volts) {
    climbIO.setRightVolts(volts);
  }

  /** Sets the desired position */
  public void setAngle(Rotation2d leftDesiredAngle, Rotation2d rightDesiredAngle) {
    leftAngleSetpoint = leftDesiredAngle;
    rightAngleSetpoint = rightDesiredAngle;

    leftClimbFeedback.reset(climbIOInputs.leftPosition.getDegrees());
    rightClimbFeedback.reset(climbIOInputs.rightPosition.getDegrees());
  }

  /** Stops both motors */
  public void stopMotors() {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  /** Returns the error of the left feedback */
  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Error")
  public double getLeftError() {
    return leftClimbFeedback.getPositionError();
  }

  /** Returns the error of the right feedback */
  @AutoLogOutput(key = "Climb/RightArm/Feedback/Error")
  public double getRightError() {
    return rightClimbFeedback.getPositionError();
  }

  /** Returns the setpoint of the left feedback */
  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Setpoint")
  public double getLeftSetpoint() {
    if (leftClimbFeedback.getSetpoint() == null) {
      return 0.0;
    }
    return leftClimbFeedback.getSetpoint().position;
  }

  /** Returns the setpoint of the right feedback */
  @AutoLogOutput(key = "Climb/RightArm/Feedback/Setpoint")
  public double getRightSetpoint() {
    if (rightClimbFeedback.getSetpoint() == null) {
      return 0.0;
    }
    return rightClimbFeedback.getSetpoint().position;
  }

  /** Returns if the left feedback is at the setpoint */
  @AutoLogOutput(key = "Climb/LeftArm/Feedback/isAtSetpoint")
  public boolean isLeftAtSetpoint() {
    return leftClimbFeedback.atSetpoint();
  }

  /** Returns if the right feedback is at the setpoint */
  @AutoLogOutput(key = "Climb/RightArm/Feedback/isAtSetpoint")
  public boolean isRightAtSetpoint() {
    return rightClimbFeedback.atSetpoint();
  }
}
