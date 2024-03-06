// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem */
public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Double velocitySetpointRPM = null;

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
}
