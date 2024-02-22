// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterrefactored.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Class to represent the launcher mechanism in simulation */
public class LauncherIOSim implements LauncherIO {
  private final double LOOP_PERIOD_S = 0.02;
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 2.5 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private FlywheelSim topMotor = new FlywheelSim(DCMotor.getFalcon500(1), GEARING, 0.003);
  private FlywheelSim bottomMotor = new FlywheelSim(DCMotor.getFalcon500(1), GEARING, 0.003);

  private PIDController topFeedback = new PIDController(0.019, 0.0, 0.0);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.0, 0.237);
  private SlewRateLimiter topLimiter = new SlewRateLimiter(25.0);

  private PIDController bottomFeedback = new PIDController(0.019, 0.0, 0.0);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.237);
  private SlewRateLimiter bottomLimiter = new SlewRateLimiter(25.0);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/P", topFeedback.getP());
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/I", topFeedback.getI());
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/D", topFeedback.getD());

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/P", bottomFeedback.getP());
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/I", bottomFeedback.getI());
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/D", bottomFeedback.getD());

  private LoggedTunableNumber topLimiterValue =
      new LoggedTunableNumber("Shooter/LauncherTop/Limiter/Value", 25.0);
  private LoggedTunableNumber bottomLimiterValue =
      new LoggedTunableNumber("Shooter/LauncherBottom/Limiter/Value", 25.0);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private double topVelocitySetpointMPS = 0.0;
  private double bottomVelocitySetpointMPS = 0.0;

  /** Create a new virtual implementation of the launcher */
  public LauncherIOSim() {
    topFeedback.setTolerance(10.0, 10.0);
    bottomFeedback.setTolerance(0.1, 5.0);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    topMotor.update(LOOP_PERIOD_S);
    bottomMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            topMotor.getCurrentDrawAmps() + bottomMotor.getCurrentDrawAmps()));

    inputs.topFlywheelVelocityMPS = (topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0;
    inputs.topFlywheelAppliedVolts = topAppliedVolts;
    inputs.topFlywheelAppliedCurrentAmps = new double[] {topMotor.getCurrentDrawAmps()};
    inputs.topFlywheelTemperatureCelsius = new double[] {0.0};
    inputs.topFlywheelSetpointRPM = topVelocitySetpointMPS;

    inputs.bottomFlywheelVelocityMPS =
        (bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0;
    inputs.bottomFlywheelAppliedVolts = bottomAppliedVolts;
    inputs.bottomFlywheelAppliedCurrentAmps = new double[] {bottomMotor.getCurrentDrawAmps()};
    inputs.bottomFlywheelTemperatureCelsius = new double[] {0.0};
    inputs.bottomFlywheelSetpointRPM = bottomVelocitySetpointMPS;

    updateTunableNumbers();
  }

  @Override
  public void setTopVolts(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    topMotor.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVolts(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    bottomMotor.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setTopVelocity(double velocityMPS) {
    topVelocitySetpointMPS = topLimiter.calculate(velocityMPS);

    double topFeedbackOutput =
        topFeedback.calculate(
            (topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0, topVelocitySetpointMPS);
    double topFeedforwardOutput = topFeedforward.calculate(topVelocitySetpointMPS);

    double topCombinedOutput = (topFeedbackOutput + topFeedforwardOutput) / 12.0;

    setTopVolts(topFeedbackOutput);

    Logger.recordOutput("Shooter/LauncherTop/Feedback/Output", topFeedbackOutput);
    Logger.recordOutput("Shooter/LauncherTop/Feedforward/Output", topFeedforwardOutput);
    Logger.recordOutput("Shooter/LauncherTop/Combined", topCombinedOutput);
    Logger.recordOutput(
        "Shooter/LauncherTop/Feedback/VelocityError", topFeedback.getPositionError());
    Logger.recordOutput(
        "Shooter/LauncherTop/Feedback/AccelerationError", topFeedback.getVelocityError());
    Logger.recordOutput("Shooter/LauncherTop/Feedback/Setpoint", topFeedback.getSetpoint());
    Logger.recordOutput("Shooter/LauncherTop/Feedback/AtSetpoint", topFeedback.atSetpoint());
  }

  @Override
  public void setBottomVelocity(double velocityMPS) {
    bottomVelocitySetpointMPS = bottomLimiter.calculate(velocityMPS);

    double bottomFeedbackOutput =
        bottomFeedback.calculate(
            (bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0,
            bottomVelocitySetpointMPS);
    double bottomFeedforwardOutput = bottomFeedforward.calculate(bottomVelocitySetpointMPS);

    double bottomCombinedOutput = (bottomFeedbackOutput + bottomFeedforwardOutput) * 12.0;
    setBottomVolts(bottomCombinedOutput);

    Logger.recordOutput("Shooter/LauncherBottom/Feedback/Output", bottomFeedbackOutput);
    Logger.recordOutput("Shooter/LauncherBottom/Feedforward/Output", bottomFeedforwardOutput);
    Logger.recordOutput("Shooter/LauncherBottom/Combined", bottomCombinedOutput);
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
    if (topLimiterValue.hasChanged(hashCode()) || bottomLimiterValue.hasChanged(hashCode())) {
      topLimiter = new SlewRateLimiter(topLimiterValue.get());
      bottomLimiter = new SlewRateLimiter(bottomLimiterValue.get());
    }
  }
}
