// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.launcher.Launcher;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.managers.RobotSetpoints.AnglerSetpoints;
import frc.robot.managers.RobotSetpoints.LauncherSetpoints;

public class Shooter extends SubsystemBase {
  public static Rotation2d anglerPosition = null;

  private Angler angler;
  private Launcher launcher;

  public Shooter(AnglerIO anglerIO, LauncherIO launcherIO) {
    this.launcher = new Launcher(launcherIO);
    this.angler = new Angler(anglerIO);
  }

  @Override
  public void periodic() {
    anglerPosition = angler.getAnglerPosition();
  }

  public Command setShooterState(
      AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return angler.setAnglerCommand(anglerState).alongWith(launcher.setVelocityMPS(launcherState));
  }

  public void setMotors(AnglerSetpoints anglerGoal, LauncherSetpoints launcherGoal) {
    angler.setAnglerPosition(anglerGoal);
    launcher.setLauncherVelocityMPS(launcherGoal);
  }

  public void stopMotors(boolean stopLauncher, boolean stopAngler) {
    if(stopLauncher) launcher.stopMotors();
    if(stopAngler) angler.stopMotors();
  }

  public boolean atAllSetpoints() {
    return angler.atAnglerSetpoints() && launcher.atFlywheelSetpoints();
  }

  public Command characterizeFlywheel() {
    return characterizeFlywheel();
  }

  public Angler getAngler() {
    return angler;
  }

  public Launcher getLauncher() {
    return launcher;
  }
}