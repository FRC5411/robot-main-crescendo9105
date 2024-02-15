// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem */
public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private ProfiledPIDController intakeFeedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private LoggedTunableNumber feedbackP =
      new LoggedTunableNumber("Intake/Feedback/P", intakeFeedback.getP());
  private LoggedTunableNumber feedbackI =
      new LoggedTunableNumber("Intake/Feedback/I", intakeFeedback.getI());
  private LoggedTunableNumber feedbackD =
      new LoggedTunableNumber("Intake/Feedback/D", intakeFeedback.getD());
  private LoggedTunableNumber feedbackA =
      new LoggedTunableNumber(
          "Intake/Feedback/Accel", intakeFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber feedbackV =
      new LoggedTunableNumber(
          "Intake/Feedback/Vel", intakeFeedback.getConstraints().maxVelocity);

  private Double velocitySetpointRPM = null;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    intakeFeedback.setTolerance(50.0, 100.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stopMotor();
    }

    if (velocitySetpointRPM != null) {
      var motorOutput =
          (intakeFeedback.calculate(inputs.velocityRPM, velocitySetpointRPM)
              + intakeFeedforward.calculate(intakeFeedback.getGoal().velocity));

      io.setVolts(motorOutput);
    }

    updateTunableNumbers();
  }

  /** Checks if tunable numbers have changed, if so update controllers */
  private void updateTunableNumbers() {
    // hashCode() updates when class is changed (I think)
    if (feedbackP.hasChanged(hashCode())
        || feedbackI.hasChanged(hashCode())
        || feedbackD.hasChanged(hashCode())) {
      intakeFeedback.setP(feedbackP.get());
      intakeFeedback.setI(feedbackI.get());
      intakeFeedback.setD(feedbackD.get());
    }
    if (feedbackA.hasChanged(hashCode()) || feedbackV.hasChanged(hashCode())) {
      var newConstraints = new TrapezoidProfile.Constraints(feedbackA.get(), feedbackV.get());

      intakeFeedback.setConstraints(newConstraints);
    }
  }

  /** Sets the desired velocity in RPM */
  public void setVelocity(double desiredVelocityRPM) {
    velocitySetpointRPM = desiredVelocityRPM;

    Logger.recordOutput("Intake/Feedback/Setpoint", velocitySetpointRPM);
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  /** Stops the motor */
  public void stopMotor() {
    io.setVolts(0.0);
  }

  /** Returns the intake motor's velocity */
  @AutoLogOutput(key = "Intake/IntakeMotor/Velocity")
  public double getVelocity() {
    return inputs.velocityRPM;
  }

  /** Returns the bus voltage from the intake */
  @AutoLogOutput(key = "Intake/IntakeMotor/AppliedVolts")
  public double[] getAppliedVolts() {
    return inputs.appliedCurrentAmps;
  }

  /** Returns if the controller is at the setpoint */
  @AutoLogOutput(key = "Intake/Feedback/AtSetpoint")
  public boolean isAtSetpoint() {
    return intakeFeedback.atSetpoint();
  }

  /** Returns the controller's velocity error */
  @AutoLogOutput(key = "Intake/Feedback/VelocityError")
  public double getFeedbackError() {
    return intakeFeedback.getVelocityError();
  }
}
