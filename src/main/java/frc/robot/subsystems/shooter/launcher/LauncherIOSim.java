// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Class to represent the launcher in simulation */
public class LauncherIOSim implements LauncherIO {
  // TODO Update as needed
  private double LOOP_PERIOD_S = 0.02;
  private final double RADIUS_METERS = 7.62 / 100.0;
  private final double CIRCUMFRENCE_METERS = 2.0 * Math.PI * RADIUS_METERS;

  private FlywheelSim topMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.0002);
  private FlywheelSim bottomMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.0002);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private PIDController topFeedback = new PIDController(0.0, 0.0, 0.0);
  private PIDController bottomFeedback = new PIDController(0.0, 0.0, 0.0);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/P", topFeedback.getP());
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/I", topFeedback.getI());
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/D", topFeedback.getD());

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/P", bottomFeedback.getP());
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/I", bottomFeedback.getI());
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/D", bottomFeedback.getD());

  /** Create a new virtual implementation of the launcher */
  public LauncherIOSim() {}

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    topMotor.update(LOOP_PERIOD_S);
    bottomMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(topMotor.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(bottomMotor.getCurrentDrawAmps()));

    inputs.topAngleRadians = 0.0;
    inputs.topVelocityRPM = topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS / 60.0;
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getCurrentDrawAmps()};
    inputs.topTemperatureCelsius = new double[] {0.0};

    inputs.bottomAngleRadians = 0.0;
    inputs.bottomVelocityRPM = bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS / 60.0;
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps = new double[] {bottomMotor.getCurrentDrawAmps()};
    inputs.bottomTemperatureCelsius = new double[] {0.0};

    updateTunableNumbers();
  }

  @Override
  public void setVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    topMotor.setInputVoltage(topAppliedVolts);
    bottomMotor.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setVelocity(double velocityMPS) {
    double topVelocity = topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS / 60.0;
    double bottomVelocity = bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_METERS / 60.0;

    var topFeedforwardOutput = topFeedforward.calculate(topVelocity);
    var bottomFeedforwardOutput = bottomFeedforward.calculate(bottomVelocity);

    var topFeedbackOutput = topFeedback.calculate(topVelocity, velocityMPS);
    var bottomFeedbackOutput = bottomFeedback.calculate(bottomVelocity, velocityMPS);

    topMotor.setInput(topFeedbackOutput + topFeedforwardOutput);
    bottomMotor.setInput(bottomFeedbackOutput + bottomFeedforwardOutput);

    Logger.recordOutput("Shooter/LauncherIO/TopRoller/Feedback/Output", topFeedbackOutput);
    Logger.recordOutput("Shooter/LauncherIO/BottomRoller/Feedforward/kOutput", bottomFeedbackOutput);
    Logger.recordOutput("Shooter/LauncherIO/TopRoller/Feedback/Output", topFeedforwardOutput);
    Logger.recordOutput("Shooter/LauncherIO/BottomRoller/Feedforward/Output", bottomFeedforwardOutput);
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
