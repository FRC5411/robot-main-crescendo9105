// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
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

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Inputs", inputs);
  }

  /** Checks if tunable numbers have changed, if so update controllers */
  private void updateTunableNumbers() {
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
}
