// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
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
      new LoggedTunableNumber("Intake/Tuning/P", intakeFeedback.getP());
  private LoggedTunableNumber feedbackI =
      new LoggedTunableNumber("Intake/Tuning/I", intakeFeedback.getI());
  private LoggedTunableNumber feedbackD =
      new LoggedTunableNumber("Intake/Tuning/D", intakeFeedback.getD());
  private LoggedTunableNumber feedbackA =
      new LoggedTunableNumber("Intake/Tuning/A", intakeFeedback.getConstraints().maxAcceleration);
  private LoggedTunableNumber feedbackV =
      new LoggedTunableNumber("Intake/Tuning/V", intakeFeedback.getConstraints().maxVelocity);

  private Measure<Velocity<Angle>> feedbackSetpoint = null;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stopMotor();
    }

    if (feedbackSetpoint != null) {
      var motorOutput =
          (intakeFeedback.calculate(inputs.velocity.magnitude(), feedbackSetpoint.magnitude())
                  + intakeFeedforward.calculate(feedbackSetpoint.magnitude()))
              * 12.0; // Multiply by 12 since output is volts

      Logger.recordOutput("Intake/Controller/Output", motorOutput);
      io.setVolts(Units.Volts.of(motorOutput));
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
  public void setVelocity(Measure<Velocity<Angle>> desiredVelocity) {
    feedbackSetpoint = desiredVelocity;

    Logger.recordOutput("Intake/Controller/Setpoint", desiredVelocity);
  }

  /** Stops the motor */
  public void stopMotor() {
    io.setVolts(Units.Volts.of(0.0));
  }

  /** Returns the intake motor's velocity */
  @AutoLogOutput(key = "Intake/Velocity")
  public Measure<Velocity<Angle>> getVelocity() {
    return inputs.velocity;
  }

  /** Returns the bus voltage from the intake */
  @AutoLogOutput(key = "Intake/AppliedVolts")
  public Measure<Voltage> getAppliedVolts() {
    return inputs.appliedVolts;
  }
}
