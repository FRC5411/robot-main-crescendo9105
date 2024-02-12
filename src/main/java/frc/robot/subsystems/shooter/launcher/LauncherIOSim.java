// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Class to represent the launcher in simulation */
public class LauncherIOSim implements LauncherIO {
  // TODO Update as needed
  private double LOOP_PERIOD_S = 0.02;

  private FlywheelSim topMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 1.0);
  private FlywheelSim bottomtMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 1.0);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  /** Create a new virtual implementation of the launcher */
  public LauncherIOSim() {}

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    topMotor.update(LOOP_PERIOD_S);
    bottomtMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(topMotor.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(bottomtMotor.getCurrentDrawAmps()));

    inputs.topAngleRadians = 0.0;
    inputs.topVelocityRPM = topMotor.getAngularVelocityRPM();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getCurrentDrawAmps()};
    inputs.topTemperatureCelsius = new double[] {0.0};

    inputs.bottomAngleRadians = 0.0;
    inputs.bottomVelocityRPM = bottomtMotor.getAngularVelocityRPM();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps = new double[] {bottomtMotor.getCurrentDrawAmps()};
    inputs.bottomTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    topMotor.setInputVoltage(topVolts);
    bottomtMotor.setInputVoltage(bottomVolts);
  }
}
