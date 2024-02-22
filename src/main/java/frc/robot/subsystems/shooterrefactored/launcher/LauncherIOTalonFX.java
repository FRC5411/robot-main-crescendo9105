// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterrefactored.launcher;

/** Class to interact with the physical launcher structure */
public class LauncherIOTalonFX implements LauncherIO {

  /** Create a new hardware implementation of the launcher */
  public LauncherIOTalonFX() {}

  @Override
  public void updateInputs(LauncherIOInputs inputs) {}

  @Override
  public void setTopVolts(double volts) {}

  @Override
  public void setBottomVolts(double volts) {}

  @Override
  public void setTopVelocity(double velocityMPS) {}

  @Override
  public void setBottomVelocity(double velocityMPS) {}
}
