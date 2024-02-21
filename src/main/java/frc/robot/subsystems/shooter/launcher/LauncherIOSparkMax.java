// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to interact with the physical launcher */
public class LauncherIOSparkMax implements LauncherIO {
  private final double RADIUS_M = 7.62 / 100.0;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private CANSparkMax topMotor = new CANSparkMax(43, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(44, MotorType.kBrushless);

  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private SparkPIDController topFeedback = topMotor.getPIDController();
  private SparkPIDController bottomFeedback = bottomMotor.getPIDController();

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/P", 0.0);
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/I", 0.0);
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/D", 0.0);

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/P", 0.0);
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/I", 0.0);
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/D", 0.0);

  /** Create a new hardware implementation of the launcher */
  public LauncherIOSparkMax() {
    // TODO Update as needed
    topMotor.clearFaults();
    bottomMotor.clearFaults();
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setSmartCurrentLimit(40);
    bottomMotor.setSmartCurrentLimit(40);
    topMotor.enableVoltageCompensation(12.0);
    bottomMotor.enableVoltageCompensation(12.0);
    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setIdleMode(IdleMode.kBrake);

    topMotor.setInverted(true);
    bottomMotor.setInverted(false);

    topEncoder.setPosition(0.0);
    bottomEncoder.setPosition(0.0);

    topFeedback.setP(topFeedbackP.get());
    topFeedback.setI(topFeedbackI.get());
    topFeedback.setD(topFeedbackD.get());

    bottomFeedback.setP(bottomFeedbackP.get());
    bottomFeedback.setI(bottomFeedbackI.get());
    bottomFeedback.setD(bottomFeedbackD.get());

    topFeedback.setFeedbackDevice(topEncoder);
    bottomFeedback.setFeedbackDevice(bottomEncoder);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void updateInputs(LauncherIOInputsI inputs) {
    // TODO Update as needed
    inputs.topAngleRadians = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getOutputCurrent()};
    inputs.topTemperatureCelsius = new double[] {topMotor.getMotorTemperature()};

    inputs.bottomAngleRadians = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps = new double[] {bottomMotor.getOutputCurrent()};
    inputs.bottomTemperatureCelsius = new double[] {bottomMotor.getMotorTemperature()};

    updateTunableNumbers();
  }

  @Override
  public void setVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    topMotor.setVoltage(topAppliedVolts);
    bottomMotor.setVoltage(bottomAppliedVolts);
  }

  @Override
  public void setVelocity(double velocityMPS) {
    var topFeedforwardOutput = topFeedforward.calculate(topEncoder.getVelocity() / CIRCUMFRENCE_M);
    var bottomFeedforwardOutput =
        bottomFeedforward.calculate(bottomEncoder.getVelocity() / CIRCUMFRENCE_M);

    topFeedback.setReference(
        60.0 * (velocityMPS / CIRCUMFRENCE_M), ControlType.kVelocity, 0, topFeedforwardOutput);
    bottomFeedback.setReference(
        60.0 * (velocityMPS / CIRCUMFRENCE_M), ControlType.kVelocity, 0, bottomFeedforwardOutput);
  }

  /** Update the tunable numbers if they've changed */
  private void updateTunableNumbers() {
    if (topFeedbackP.hasChanged(hashCode())
        || topFeedbackI.hasChanged(hashCode())
        || topFeedbackD.hasChanged(hashCode())) {
      topFeedback.setP(topFeedbackP.get());
      topFeedback.setI(topFeedbackI.get());
      topFeedback.setD(topFeedbackD.get());
    }
    if (bottomFeedbackP.hasChanged(hashCode())
        || bottomFeedbackI.hasChanged(hashCode())
        || bottomFeedbackD.hasChanged(hashCode())) {
      bottomFeedback.setP(bottomFeedbackP.get());
      bottomFeedback.setI(bottomFeedbackI.get());
      bottomFeedback.setD(bottomFeedbackD.get());
    }
  }
}
