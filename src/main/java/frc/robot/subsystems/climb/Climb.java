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
import frc.robot.RobotStates.ClimbStates;
import frc.robot.subsystems.climb.ClimbVisualizer.ClimbSide;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

/** Climb subsystem */
public class Climb extends SubsystemBase {
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

  private double climbDirection = 1.0;

  private ClimbIO climbIO;
  private ClimbIOInputsAutoLogged climbIOInputs = new ClimbIOInputsAutoLogged();

  private PIDController leftClimbFeedback = new PIDController(0.0, 0.0, 0.0);
  private PIDController rightClimbFeedback = new PIDController(0.0, 0.0, 0.0);

  // private ProfiledPIDController leftClimbFeedback =
  //     new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(100.0, 100.0));
  // private ProfiledPIDController rightClimbFeedback =
  //     new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(100.0, 100.0));

  // private ArmFeedforward leftClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  // private ArmFeedforward rightClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  private LoggedTunableNumber leftFeedbackP;
  private LoggedTunableNumber leftFeedbackI;
  private LoggedTunableNumber leftFeedbackD;
  // private LoggedTunableNumber leftFeedbackA;
  // private LoggedTunableNumber leftFeedbackV;

  private LoggedTunableNumber rightFeedbackP;
  private LoggedTunableNumber rightFeedbackI;
  private LoggedTunableNumber rightFeedbackD;
  // private LoggedTunableNumber rightFeedbackA;
  // private LoggedTunableNumber rightFeedbackV;

  private Rotation2d leftAngleSetpoint = null;
  private Rotation2d rightAngleSetpoint = null;

  private ClimbVoltSetpoints currentSetpoint = null;

  private ClimbVisualizer leftVisualizer = new ClimbVisualizer(ClimbSide.LEFT);
  private ClimbVisualizer rightVisualizer = new ClimbVisualizer(ClimbSide.RIGHT);

  private Rotation2d minAngle = Rotation2d.fromDegrees(-30.0);
  private Rotation2d maxAngle = Rotation2d.fromDegrees(150.0);

  /** Creates a new Climb. */
  public Climb(ClimbIO io) {
    this.climbIO = io;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          leftClimbFeedback.setP(0.1);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          // leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          rightClimbFeedback.setP(0.1);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          // rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          break;
        case SIM:
          leftClimbFeedback.setP(1.0);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          // leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          rightClimbFeedback.setP(1.0);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          // rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(100.0, 100.0));

          break;
        default:
          leftClimbFeedback.setP(0.0);
          leftClimbFeedback.setI(0.0);
          leftClimbFeedback.setD(0.0);
          // leftClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          rightClimbFeedback.setP(0.0);
          rightClimbFeedback.setI(0.0);
          rightClimbFeedback.setD(0.0);
          // rightClimbFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          break;
      }
    }

    leftFeedbackP = new LoggedTunableNumber("Climb/LeftArm/Feedback/P", leftClimbFeedback.getP());
    leftFeedbackI = new LoggedTunableNumber("Climb/LeftArm/Feedback/I", leftClimbFeedback.getI());
    leftFeedbackD = new LoggedTunableNumber("Climb/LeftArm/Feedback/D", leftClimbFeedback.getD());
    // leftFeedbackA =
    //     new LoggedTunableNumber(
    //         "Climb/LeftArm/Feedback/V", leftClimbFeedback.getConstraints().maxVelocity);
    // leftFeedbackV =
    //     new LoggedTunableNumber(
    //         "Climb/LeftArm/Feedback/A", leftClimbFeedback.getConstraints().maxAcceleration);

    rightFeedbackP =
        new LoggedTunableNumber("Climb/RightArm/Feedback/P", rightClimbFeedback.getP());
    rightFeedbackI =
        new LoggedTunableNumber("Climb/RightArm/Feedback/I", rightClimbFeedback.getI());
    rightFeedbackD =
        new LoggedTunableNumber("Climb/RightArm/Feedback/D", rightClimbFeedback.getD());
    // rightFeedbackA =
    //     new LoggedTunableNumber(
    //         "Climb/RightArm/Feedback/V", rightClimbFeedback.getConstraints().maxVelocity);
    // rightFeedbackV =
    //     new LoggedTunableNumber(
    //         "Climb/RightArm/Feedback/A", rightClimbFeedback.getConstraints().maxAcceleration);

    leftClimbFeedback.setTolerance(2.0);
    rightClimbFeedback.setTolerance(2.0);

    leftClimbFeedback.enableContinuousInput(-180.0, 180.0);
    rightClimbFeedback.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOInputs);
    Logger.processInputs("Climb/Inputs", climbIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (currentSetpoint != null) {
      climbIO.setLeftVolts(currentSetpoint.leftVolts);
      climbIO.setRightVolts(currentSetpoint.rightVolts);
    }

    // if (leftAngleSetpoint != null) {
    //   double leftFeedbackOutput =
    //       leftClimbFeedback.calculate(
    //           climbIOInputs.leftPosition.getDegrees(),
    //           MathUtil.clamp(
    //               leftAngleSetpoint.getDegrees(), minAngle.getDegrees(), maxAngle.getDegrees()));

    //   Logger.recordOutput("Climb/LeftArm/Feedback/Output", leftFeedbackOutput);

    //   climbIO.setLeftVolts(leftFeedbackOutput);
    // }

    // if (rightAngleSetpoint != null) {
    //   double rightFeedbackOutput =
    //       rightClimbFeedback.calculate(
    //           climbIOInputs.rightPosition.getDegrees(),
    //           MathUtil.clamp(
    //               rightAngleSetpoint.getDegrees(), minAngle.getDegrees(),
    // maxAngle.getDegrees()));

    //   Logger.recordOutput("Climb/RightArm/Feedback/Output", rightFeedbackOutput);

    //   climbIO.setRightVolts(rightFeedbackOutput);
    // }

    if (currentSetpoint != null) {
      if (climbIOInputs.leftPosition.getDegrees() > 75.0 && currentSetpoint.leftVolts > 0.0) {
        climbIO.setLeftVolts(0.0);
      } else if (climbIOInputs.leftPosition.getDegrees() < -85.0
          && currentSetpoint.leftVolts < 0.0) {
        climbIO.setLeftVolts(0.0);
      }

      if (climbIOInputs.rightPosition.getDegrees() > 75.0 && currentSetpoint.rightVolts > 0.0) {
        climbIO.setRightVolts(0.0);
      } else if (climbIOInputs.rightPosition.getDegrees() < -85.0
          && currentSetpoint.rightVolts < 0.0) {
        climbIO.setRightVolts(0.0);
      }
    }

    leftVisualizer.updateClimbAngle(climbIOInputs.leftPosition);
    rightVisualizer.updateClimbAngle(climbIOInputs.rightPosition);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
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

    if (rightFeedbackP.hasChanged(hashCode())
        || rightFeedbackI.hasChanged(hashCode())
        || rightFeedbackD.hasChanged(hashCode())) {
      rightClimbFeedback.setP(rightFeedbackP.get());
      rightClimbFeedback.setI(rightFeedbackI.get());
      rightClimbFeedback.setD(rightFeedbackD.get());
    }
  }

  public HashMap<ClimbStates, Command> mapToCommand() {
    HashMap<ClimbStates, Command> commandMap = new HashMap<>();

    commandMap.put(ClimbStates.IDLE, setPositionSetpoint(ClimPosSetpoints.IDLE.getRotation()));
    commandMap.put(ClimbStates.OFF, setManualVolts(ClimbVoltSetpoints.OFF));
    commandMap.put(ClimbStates.MOVE_BOTH_UP, setManualVolts(ClimbVoltSetpoints.BOTH_UP));
    commandMap.put(ClimbStates.MOVE_BOTH_DOWN, setManualVolts(ClimbVoltSetpoints.BOTH_DOWN));
    commandMap.put(ClimbStates.AMP, setPositionSetpoint(ClimPosSetpoints.AMP.getRotation()));

    return commandMap;
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

  public Command setManualVolts(ClimbVoltSetpoints setpoint) {
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

  // public void setAngle(Rotation2d leftDesiredAngle, Rotation2d rightDesiredAngle) {
  //   leftAngleSetpoint = leftDesiredAngle;
  //   rightAngleSetpoint = rightDesiredAngle;

  //   leftClimbFeedback.reset(climbIOInputs.leftPosition.getDegrees());
  //   rightClimbFeedback.reset(climbIOInputs.rightPosition.getDegrees());
  // }

  public void stopMotors() {
    // leftAngleSetpoint = null;
    // rightAngleSetpoint = null;

    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  // @AutoLogOutput(key = "Climb/LeftArm/Feedback/Error")
  // public double getLeftError() {
  //   return leftClimbFeedback.getPositionError();
  // }

  // @AutoLogOutput(key = "Climb/RightArm/Feedback/Error")
  // public double getRightError() {
  //   return rightClimbFeedback.getPositionError();
  // }

  // @AutoLogOutput(key = "Climb/LeftArm/Feedback/Setpoint")
  // public double getLeftSetpoint() {
  //   if (leftClimbFeedback.getSetpoint() == null) {
  //     return 0.0;
  //   }
  //   return Math.toRadians(leftClimbFeedback.getSetpoint().position);
  // }

  // @AutoLogOutput(key = "Climb/LeftArm/Feedback/Goal")
  // public double getLeftGoal() {
  //   return (leftClimbFeedback.getGoal() == null)
  //       ? 0.0
  //       : Math.toRadians(leftClimbFeedback.getGoal().position);
  // }

  // @AutoLogOutput(key = "Climb/RightArm/Feedback/Setpoint")
  // public double getRightSetpoint() {
  //   return (rightClimbFeedback.getSetpoint() == null)
  //       ? 0.0
  //       : Math.toRadians(rightClimbFeedback.getSetpoint().position);
  // }

  // @AutoLogOutput(key = "Climb/LeftArm/Feedback/Goal")
  // public double getRightGoal() {
  //   return (leftClimbFeedback.getGoal() == null)
  //       ? 0.0
  //       : Math.toRadians(leftClimbFeedback.getGoal().position);
  // }

  // @AutoLogOutput(key = "Climb/LeftArm/Feedback/isAtSetpoint")
  // public boolean isLeftAtSetpoint() {
  //   return leftClimbFeedback.atSetpoint();
  // }

  // @AutoLogOutput(key = "Climb/RightArm/Feedback/isAtSetpoint")
  // public boolean isRightAtSetpoint() {
  //   return rightClimbFeedback.atSetpoint();
  // }
}
