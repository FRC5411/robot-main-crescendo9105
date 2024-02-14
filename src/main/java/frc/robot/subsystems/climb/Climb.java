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
import frc.robot.subsystems.climb.ClimbVisualizer.climbSide;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Climb subsystem */
public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private ProfiledPIDController leftClimbFeedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private ProfiledPIDController rightClimbFeedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  private ArmFeedforward leftClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private ArmFeedforward rightClimbFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  private LoggedTunableNumber leftFeedbackP =
      new LoggedTunableNumber("Climb/Tuning/Left/P", leftClimbFeedback.getP());
  private LoggedTunableNumber leftFeedbackI =
      new LoggedTunableNumber("Climb/Tuning/Left/I", leftClimbFeedback.getI());
  private LoggedTunableNumber leftFeedbackD =
      new LoggedTunableNumber("Climb/Tuning/Left/D", leftClimbFeedback.getD());
  private LoggedTunableNumber leftFeedbackA =
      new LoggedTunableNumber(
          "Climb/Tuning/Left/Accel", leftClimbFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber leftFeedbackV =
      new LoggedTunableNumber(
          "Climb/Tuning/Left/Vel", leftClimbFeedback.getConstraints().maxVelocity);

  private LoggedTunableNumber rightFeedbackP =
      new LoggedTunableNumber("Climb/Tuning/Right/P", rightClimbFeedback.getP());
  private LoggedTunableNumber rightFeedbackI =
      new LoggedTunableNumber("Climb/Tuning/Right/I", rightClimbFeedback.getI());
  private LoggedTunableNumber rightFeedbackD =
      new LoggedTunableNumber("Climb/Tuning/Right/D", rightClimbFeedback.getD());
  private LoggedTunableNumber rightFeedbackA =
      new LoggedTunableNumber(
          "Climb/Tuning/Right/Accel", rightClimbFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber rightFeedbackV =
      new LoggedTunableNumber(
          "Climb/Tuning/Right/Vel", rightClimbFeedback.getConstraints().maxVelocity);

  private Double leftAngleSetpointRadians = null;
  private Double rightAngleSetpointRadians = null;

  private ClimbVisualizer leftVisualizer = new ClimbVisualizer(climbSide.LEFT);
  private ClimbVisualizer rightVisualizer = new ClimbVisualizer(climbSide.RIGHT);

  /** Creates a new Climb. */
  public Climb(ClimbIO io) {
    this.io = io;

    leftClimbFeedback.setTolerance(0.2);
    rightClimbFeedback.setTolerance(0.2);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (leftAngleSetpointRadians != null) {
      var leftFeedbackOutput =
          leftClimbFeedback.calculate(inputs.leftAngleRadians, leftAngleSetpointRadians);
      // TODO Need to add angle offset
      var leftFeedforwardOutput =
          leftClimbFeedforward.calculate(
              inputs.leftAngleRadians, leftClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/Controller/leftFeedbackOutput", leftFeedbackOutput);
      Logger.recordOutput("Climb/Controller/leftFeedforwardOutput", leftFeedforwardOutput);

      io.setLeftVolts((leftFeedbackOutput + leftFeedforwardOutput) * 12.0);
    }

    if (rightAngleSetpointRadians != null) {
      var rightFeedbackOutput =
          rightClimbFeedback.calculate(inputs.rightAngleRadians, rightAngleSetpointRadians);
      // TODO Need to add angle offset
      var rightFeedforwardOutput =
          rightClimbFeedforward.calculate(
              inputs.rightAngleRadians, rightClimbFeedback.getSetpoint().velocity);

      Logger.recordOutput("Climb/Controller/rightFeedbackOutput", rightFeedbackOutput);
      Logger.recordOutput("Climb/Controller/rightFeedforwardOutput", rightFeedforwardOutput);

      io.setRightVolts((rightFeedbackOutput + rightFeedforwardOutput) * 12.0);
    }

    updateTunableNumbers();

    leftVisualizer.updateClimbAngle(Rotation2d.fromRadians(inputs.leftAngleRadians));
    rightVisualizer.updateClimbAngle(Rotation2d.fromRadians(inputs.rightAngleRadians));
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

  /** Sets the desired position in Radians */
  public void setAngle(double leftDesiredAngleRadians, double rightDesiredAngleRadians) {
    leftAngleSetpointRadians = leftDesiredAngleRadians;
    rightAngleSetpointRadians = rightDesiredAngleRadians;

    Logger.recordOutput("Climb/Controller/leftSetpoint", leftAngleSetpointRadians);
    Logger.recordOutput("Climb/Controller/rightSetpoint", rightAngleSetpointRadians);
  }

  /** Stops both motors */
  public void stopMotors() {
    leftAngleSetpointRadians = 0.0;
    rightAngleSetpointRadians = 0.0;

    io.setLeftVolts(0.0);
    io.setRightVolts(0.0);
  }

  /** Returns the error of the left feedback */
  @AutoLogOutput(key = "Climb/Controller/Left/Error")
  public double getLeftError() {
    return leftClimbFeedback.getPositionError();
  }

  /** Returns the error of the right feedback */
  @AutoLogOutput(key = "Climb/Controller/Right/Setpoint")
  public double getRightError() {
    return rightClimbFeedback.getPositionError();
  }

  /** Returns the setpoint of the left feedback */
  @AutoLogOutput(key = "Climb/Controller/Left/Setpoint")
  public double getLeftSetpoint() {
    return leftClimbFeedback.getSetpoint().position;
  }

  /** Returns the setpoint of the right feedback */
  @AutoLogOutput(key = "Climb/Controller/Right/Setpoint")
  public double getRightSetpoint() {
    return rightClimbFeedback.getSetpoint().position;
  }

  /** Returns if the left feedback is at the setpoint */
  @AutoLogOutput(key = "Climb/Controller/Left/isAtSetpoint")
  public boolean isLeftAtSetpoint() {
    return leftClimbFeedback.atSetpoint();
  }

  /** Returns if the right feedback is at the setpoint */
  @AutoLogOutput(key = "Climb/Controller/Right/Setpoint")
  public boolean isRightAtSetpoint() {
    return rightClimbFeedback.atSetpoint();
  }
}