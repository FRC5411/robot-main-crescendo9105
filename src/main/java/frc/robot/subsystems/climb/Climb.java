// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

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

  /** Creates a new Climb. */
  public Climb(ClimbIO io) {
    this.io = io;

    leftClimbFeedback.setTolerance(0.2);
    rightClimbFeedback.setTolerance(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
