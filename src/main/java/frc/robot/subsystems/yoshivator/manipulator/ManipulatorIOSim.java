// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator.manipulator;

/** Class to represent the manipulator mechanism in simulation */
public class ManipulatorIOSim implements ManipulatorIO {
  /** Create a new virtual implementation of the manipulator */
  public ManipulatorIOSim() {}

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {}

  @Override
  public void setPivotVolts(double volts) {}

  @Override
  public void setFlywheelVolts(double volts) {}
}
