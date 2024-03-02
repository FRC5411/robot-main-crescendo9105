// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

/** Class to represent the yoshi mechanism in simulation */
public class YoshivatorIOSim implements YoshivatorIO {

  /** Create a new virtual implementation of the yoshi */
  public YoshivatorIOSim() {}

  @Override
  public void updateInputs(YoshivatorIOInputs inputs) {}

  @Override
  public void setPivotVolts(double volts) {}

  @Override
  public void setIntakeVolts(double volts) {}
}
