// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Class to represent the intake mechanism in simulation */
public class IntakeIOSim implements IntakeIO {
  private final double LOOP_PERIOD_S = 0.02;

  // TODO Update values to reflect real world as needed
  private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1), 1.0, 1.0);

  /** Create a new virtual implementation of the intake */
  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeMotor.getCurrentDrawAmps()));

    inputs.angleRotations = 0.0; // Flywheel sim doesn't have pose methods
    inputs.velocityRPM = intakeMotor.getAngularVelocityRPM();
    inputs.appliedVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            intakeMotor.getCurrentDrawAmps()); // Might be broken lol
    inputs.appliedCurrentAmps = new double[] {intakeMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    var adjustedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeMotor.setInputVoltage(adjustedVolts);
  }
}
