// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.climb.ClimbVisualizer.ClimbSide;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.managers.RobotSetpoints.ClimbVoltSetpoints;

public class Climb extends SubsystemBase {
<<<<<<< HEAD
=======
  public static enum ClimbVoltSetpoints {
    // LEFT(9.0, 0.0),
    // RIGHT(0.0, 9.0),
    BOTH_UP(12.0, 12.0),
    BOTH_DOWN(-12.0, -12.0),
    OFF(0.0, 0.0);

    private double leftVolts;
    private double rightVolts;

    ClimbVoltSetpoints(double leftVolts, double rightVolts) {
      this.leftVolts = leftVolts;
      this.rightVolts = rightVolts;
    }
  }

  public static enum ClimPosSetpoints {
    AMP(Rotation2d.fromDegrees(110.0)),
    IDLE(Rotation2d.fromDegrees(5.0));

    private Rotation2d m_setpoint;

    ClimPosSetpoints(Rotation2d setpoint) {
      this.m_setpoint = setpoint;
    }

    private Rotation2d getRotation() {
      return m_setpoint;
    }
  }

>>>>>>> a57434dd4aadf1dde92d3e1ce94934e3d87c44b0
  private ClimbIO climbIO;
  private ClimbIOInputsAutoLogged climbIOInputs = new ClimbIOInputsAutoLogged();

  private PIDController leftClimbFeedback = new PIDController(0.1, 0.0, 0.0);
  private PIDController rightClimbFeedback = new PIDController(0.1, 0.0, 0.0);

  private LoggedTunableNumber leftFeedbackP;
  private LoggedTunableNumber leftFeedbackI;
  private LoggedTunableNumber leftFeedbackD;

  private LoggedTunableNumber rightFeedbackP;
  private LoggedTunableNumber rightFeedbackI;
  private LoggedTunableNumber rightFeedbackD;

  private Rotation2d leftAngleSetpoint = null;
  private Rotation2d rightAngleSetpoint = null;

  private ClimbVoltSetpoints currentSetpoint = null;

  private ClimbVisualizer leftVisualizer = new ClimbVisualizer(ClimbSide.LEFT);
  private ClimbVisualizer rightVisualizer = new ClimbVisualizer(ClimbSide.RIGHT);

  private Rotation2d minAngle = Rotation2d.fromDegrees(-85.0);
  private Rotation2d maxAngle = Rotation2d.fromDegrees(75.0);

  public Climb(ClimbIO io) {
    this.climbIO = io;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          leftClimbFeedback.setPID(0.1, 0.0, 0.0);
          rightClimbFeedback.setPID(0.1, 0.0, 0.0);
          break;
        case SIM:
          leftClimbFeedback.setPID(1.0, 0.0, 0.0);
          rightClimbFeedback.setPID(1.0, 0.0, 0.0);
          break;
        default:
          break;
      }
    }

    leftFeedbackP = new LoggedTunableNumber("Climb/LeftArm/Feedback/P", leftClimbFeedback.getP());
    leftFeedbackI = new LoggedTunableNumber("Climb/LeftArm/Feedback/I", leftClimbFeedback.getI());
    leftFeedbackD = new LoggedTunableNumber("Climb/LeftArm/Feedback/D", leftClimbFeedback.getD());

    rightFeedbackP =
        new LoggedTunableNumber("Climb/RightArm/Feedback/P", rightClimbFeedback.getP());
    rightFeedbackI =
        new LoggedTunableNumber("Climb/RightArm/Feedback/I", rightClimbFeedback.getI());
    rightFeedbackD =
        new LoggedTunableNumber("Climb/RightArm/Feedback/D", rightClimbFeedback.getD());

    leftClimbFeedback.setTolerance(2.0);
    rightClimbFeedback.setTolerance(2.0);

    leftClimbFeedback.enableContinuousInput(-180.0, 180.0);
    rightClimbFeedback.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOInputs);
    Logger.processInputs("Climb/Inputs", climbIOInputs);

    if (DriverStation.isDisabled()) stopMotors();

    if (currentSetpoint != null) {
      if (climbIOInputs.leftPosition.getDegrees() > maxAngle.getDegrees() 
        && currentSetpoint.getLeftVolts() > 0.0) {
        climbIO.setLeftVolts(0.0);
      } else if (climbIOInputs.leftPosition.getDegrees() < minAngle.getDegrees()
          && currentSetpoint.getLeftVolts() < 0.0) {
        climbIO.setLeftVolts(0.0);
      } else {
        climbIO.setLeftVolts(currentSetpoint.getLeftVolts());
      }

      if (climbIOInputs.rightPosition.getDegrees() > maxAngle.getDegrees() 
        && currentSetpoint.getRightVolts() > 0.0) {
        climbIO.setRightVolts(0.0);
      } else if (climbIOInputs.rightPosition.getDegrees() < minAngle.getDegrees()
          && currentSetpoint.getRightVolts() < 0.0) {
        climbIO.setRightVolts(0.0);
      } else {
        climbIO.setRightVolts(currentSetpoint.getRightVolts());
      }
    }

    // if (leftAngleSetpoint != null) {
    //   double leftFeedbackOutput =
    //     leftClimbFeedback.calculate(
    //       climbIOInputs.leftPosition.getDegrees(),
    //       MathUtil.clamp(
    //         leftAngleSetpoint.getDegrees(), 
    //         minAngle.getDegrees(), 
    //         maxAngle.getDegrees()));

    //   Logger.recordOutput("Climb/LeftArm/Feedback/Output", leftFeedbackOutput);

    //   climbIO.setLeftVolts(leftFeedbackOutput);
    // }

    // if (rightAngleSetpoint != null) {
    //   double rightFeedbackOutput =
    //     rightClimbFeedback.calculate(
    //       climbIOInputs.rightPosition.getDegrees(),
    //       MathUtil.clamp(
    //          rightAngleSetpoint.getDegrees(), 
    //          minAngle.getDegrees(),
    //          maxAngle.getDegrees()));

    //   Logger.recordOutput("Climb/RightArm/Feedback/Output", rightFeedbackOutput);

    //   climbIO.setRightVolts(rightFeedbackOutput);
    // }

    leftVisualizer.updateClimbAngle(climbIOInputs.leftPosition);
    rightVisualizer.updateClimbAngle(climbIOInputs.rightPosition);

    if (Constants.tuningMode) {
      if (leftFeedbackP.hasChanged(hashCode())
        || leftFeedbackI.hasChanged(hashCode())
        || leftFeedbackD.hasChanged(hashCode())) {
        leftClimbFeedback.setP(leftFeedbackP.get());
        leftClimbFeedback.setI(leftFeedbackI.get());
        leftClimbFeedback.setD(leftFeedbackD.get());
      }

      if (rightFeedbackP.hasChanged(hashCode())
        || rightFeedbackI.hasChanged(hashCode())
        || rightFeedbackD.hasChanged(hashCode())) {
        rightClimbFeedback.setP(rightFeedbackP.get());
        rightClimbFeedback.setI(rightFeedbackI.get());
        rightClimbFeedback.setD(rightFeedbackD.get());
      }
    }
  }

  public Command setPositionSetpoint(Rotation2d pos) {
    return new InstantCommand(
        () -> {
          currentSetpoint = null;
          leftAngleSetpoint = pos;
          rightAngleSetpoint = pos;
        },
        this);
  }

  public Command runVolts(ClimbVoltSetpoints setpoint) {
    return Commands.runOnce(
        () -> {
          leftAngleSetpoint = null;
          rightAngleSetpoint = null;
          currentSetpoint = setpoint;
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

  public void stopMotors() {
    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  /* Nulls all desired setpoints */
  public void setVoltsManually(double leftVolts, double rightVolts) {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setLeftVolts(leftVolts);
    climbIO.setRightVolts(rightVolts);
  }

  /* Nulls all desired setpoints */
  public void setVoltsLeftManually(double volts) {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setLeftVolts(volts);
  }

  /* Nulls all desired setpoints */
  public void setVoltsRightManually(double volts) {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setRightVolts(volts);
  }

  /* Nulls all desired setpoints */
  public void stopMotorsManually() {
    leftAngleSetpoint = null;
    rightAngleSetpoint = null;

    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  public void setAngle(Rotation2d leftDesiredAngle, Rotation2d rightDesiredAngle) {
    leftAngleSetpoint = leftDesiredAngle;
    rightAngleSetpoint = rightDesiredAngle;
  }

  @AutoLogOutput(key = "Climb/LeftArm/Feedback/Error")
  public double getLeftError() {
    return leftClimbFeedback.getPositionError();
  }

  @AutoLogOutput(key = "Climb/RightArm/Feedback/Error")
  public double getRightError() {
    return rightClimbFeedback.getPositionError();
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