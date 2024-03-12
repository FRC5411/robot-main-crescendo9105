// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

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
import frc.robot.RobotStates.ClimbStates;
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

  private double climbDirection = 1;

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

    leftClimbFeedback.enableContinuousInput(-180.0, 180.0);
    rightClimbFeedback.enableContinuousInput(-180.0, 180.0);
  }

  public Command mapToCommand(ClimbStates state) {
    return switch (state) {
      case OFF -> Commands.runOnce(() -> setVolts(0, 0));
      case IDLE -> Commands.runOnce(() -> setAngle(new Rotation2d(0), new Rotation2d(0)));
      case GRAB -> Commands.runOnce(() -> setAngle(new Rotation2d(10), new Rotation2d(10)));
      case PULL -> Commands.runOnce(() -> setAngle(new Rotation2d(60), new Rotation2d(60)));
      case MOVE_LEFT -> setManualVoltsLeft(9);
      case MOVE_RIGHT -> setManualVoltsRight(9);
      case MOVE_BOTH -> setManualVolts(9, 9);
      case INVERT -> Commands.runOnce(() -> climbDirection *= -1, this);
      default -> Commands.runOnce(() -> setVolts(0, 0));
    };
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOInputs);
    Logger.processInputs("Climb/Inputs", climbIOInputs);

    if (DriverStation.isDisabled()) stopMotors();

    if (leftAngleSetpoint != null) {
      double leftFeedbackOutput =
          -leftClimbFeedback.calculate(
              climbIOInputs.leftPosition.getDegrees(), leftAngleSetpoint.getDegrees());

      double leftFeedforwardOutput =
          -leftClimbFeedforward.calculate(
              climbIOInputs.leftPosition.getRadians(), leftClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/LeftArm/Feedback/Output", leftFeedbackOutput);
      Logger.recordOutput("Climb/LeftArm/Feedforward/Output", leftFeedforwardOutput);

      climbIO.setLeftVolts(leftFeedbackOutput + leftFeedforwardOutput);
    }

    if (rightAngleSetpoint != null) {
      double rightFeedbackOutput =
          rightClimbFeedback.calculate(
              climbIOInputs.rightPosition.getDegrees(), rightAngleSetpoint.getDegrees());

      double rightFeedforwardOutput =
          rightClimbFeedforward.calculate(
              climbIOInputs.rightPosition.getRadians(), rightClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/RightArm/Feedback/Output", rightFeedbackOutput);
      Logger.recordOutput("Climb/RightArm/Feedforward/Output", rightFeedforwardOutput);

      climbIO.setRightVolts(rightFeedbackOutput + rightFeedforwardOutput);
    }

    leftVisualizer.updateClimbAngle(climbIOInputs.leftPosition);
    rightVisualizer.updateClimbAngle(climbIOInputs.rightPosition);

    if (Constants.tuningMode) updateTunableNumbers();
  }

  /** If tunable numbers have changed, it updates controllers */
  private void updateTunableNumbers() {
    if (leftFeedbackP.hasChanged(hashCode())
        || leftFeedbackI.hasChanged(hashCode())
        || leftFeedbackD.hasChanged(hashCode())) {
      leftClimbFeedback.setP(leftFeedbackP.get());
      leftClimbFeedback.setI(leftFeedbackI.get());
      leftClimbFeedback.setD(leftFeedbackD.get());
    }
    if (leftFeedbackA.hasChanged(hashCode()) || leftFeedbackV.hasChanged(hashCode())) {
      TrapezoidProfile.Constraints newConstraints =
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
      TrapezoidProfile.Constraints newConstraints =
          new TrapezoidProfile.Constraints(rightFeedbackA.get(), rightFeedbackV.get());

      rightClimbFeedback.setConstraints(newConstraints);
    }
  }

  public Command setManualVolts(double left, double right) {
    return Commands.runOnce(
        () -> {
          leftAngleSetpoint = null;
          rightAngleSetpoint = null;
          setVolts(left * climbDirection, right * climbDirection);
        },
        this);
  }

  public Command setManualVoltsLeft(double left) {
    return Commands.runOnce(
        () -> {
          leftAngleSetpoint = null;
          setVoltsLeft(left * climbDirection);
        },
        this);
  }

  public Command setManualVoltsRight(double right) {
    return Commands.runOnce(
        () -> {
          rightAngleSetpoint = null;
          setVoltsRight(right * climbDirection);
        },
        this);
  }

  public void setVolts(double leftVolts, double rightVolts) {
    climbIO.setLeftVolts(leftVolts);
    climbIO.setRightVolts(rightVolts);
  }

  public void setVoltsLeft(double volts) {
    climbIO.setLeftVolts(volts);
  }

  public void setVoltsRight(double volts) {
    climbIO.setRightVolts(volts);
  }

  public void setAngle(Rotation2d leftDesiredAngle, Rotation2d rightDesiredAngle) {
    leftAngleSetpoint = leftDesiredAngle;
    rightAngleSetpoint = rightDesiredAngle;

    leftClimbFeedback.reset(climbIOInputs.leftPosition.getDegrees());
    rightClimbFeedback.reset(climbIOInputs.rightPosition.getDegrees());
  }

  public void stopMotors() {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Error")
  public double getLeftError() {
    return leftClimbFeedback.getPositionError();
  }

  @AutoLogOutput(key = "Climb/RightArm/Feedback/Error")
  public double getRightError() {
    return rightClimbFeedback.getPositionError();
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Setpoint")
  public double getLeftSetpoint() {
    if (leftClimbFeedback.getSetpoint() == null) {
      return 0.0;
    }
    return Math.toRadians(leftClimbFeedback.getSetpoint().position);
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Goal")
  public double getLeftGoal() {
    return (leftClimbFeedback.getGoal() == null)
        ? 0.0
        : Math.toRadians(leftClimbFeedback.getGoal().position);
  }

  @AutoLogOutput(key = "Climb/RightArm/Feedback/Setpoint")
  public double getRightSetpoint() {
    return (rightClimbFeedback.getSetpoint() == null)
        ? 0.0
        : Math.toRadians(rightClimbFeedback.getSetpoint().position);
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Goal")
  public double getRightGoal() {
    return (leftClimbFeedback.getGoal() == null)
        ? 0.0
        : Math.toRadians(leftClimbFeedback.getGoal().position);
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/isAtSetpoint")
  public boolean isLeftAtSetpoint() {
    return leftClimbFeedback.atSetpoint();
  }

  @AutoLogOutput(key = "Climb/RightArm/Feedback/isAtSetpoint")
  public boolean isRightAtSetpoint() {
    return rightClimbFeedback.atSetpoint();
  }
}
