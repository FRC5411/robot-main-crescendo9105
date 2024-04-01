// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managers.TargetingSystem;
import frc.robot.managers.RobotStates.AnglerStates;
import frc.robot.managers.RobotStates.LauncherStates;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.launcher.Launcher;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private static LoggedTunableNumber angleTunableNumber = new LoggedTunableNumber("Shooter/AngleDebuggingDegrees", 0.0);
   
  public static enum AnglerSetpoints {
    AIM(() -> TargetingSystem.getInstance().getLaunchMapAngle()),
    CLIMB(() -> Rotation2d.fromDegrees(25.0)),
    INTAKE(() -> Rotation2d.fromDegrees(45.0)),
    IDLE(() -> Angler.getLastAnglerPosition()),
    PODIUM(() -> Rotation2d.fromDegrees(34.5)),
    SPEAKER(() -> Rotation2d.fromDegrees(57)),
    AMP(() -> Rotation2d.fromDegrees(54.0)),
    FEEDER(() -> Rotation2d.fromDegrees(45.0)),
    DEBUGGING(() -> Rotation2d.fromDegrees(angleTunableNumber.get()));

    private Supplier<Rotation2d> angleSupplier;

    AnglerSetpoints(Supplier<Rotation2d> angle) {
      this.angleSupplier = angle;
    }

    public Supplier<Rotation2d> getAngle() {
      return angleSupplier;
    }
  }

  public static enum LauncherSetpoints {
    EJECT(() -> 5.0, () -> 5.0),
    IDLE(() -> 9.0, () -> 9.0),
    SPEAKER_SHOT(() -> 38.0, () -> 38.0),
    FULL_SPEED(() -> 42.0, () -> 42.0),
    FEEDER(() -> 33.6, () -> 33.6),
    // 12.5: 7 / 9
    // 12.65: 08
    // 12.58: 9 / 12
    // AMP(() -> -0.1, () -> 12.54),
    // AMP(() -> -4.5, () -> 12.0): 5 / 8
    AMP(() -> -4.5, () -> 12.0),
    OFF(() -> 0.0, () -> 0.0);

    private DoubleSupplier topSpeedSupplierMPS;
    private DoubleSupplier bottomSpeedSupplierMPS;

    LauncherSetpoints(DoubleSupplier topSpeedMPS, DoubleSupplier bottomSpeedMPS) {
      this.topSpeedSupplierMPS = topSpeedMPS;
      this.bottomSpeedSupplierMPS = bottomSpeedMPS;
    }

    public DoubleSupplier getTopSpeedMPS() {
      return topSpeedSupplierMPS;
    }

    public DoubleSupplier getBottomSpeedMPS() {
      return bottomSpeedSupplierMPS;
    }
  }

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

  public Command getAnglerCommand(AnglerStates anglerState) {
    switch(anglerState) {
      case OFF:
        return Commands.runOnce(() -> angler.stopMotors(), this);
      case AIM:
        return angler.setAnglerCommand(AnglerSetpoints.AIM);
      case INTAKE:
        return angler.setAnglerCommand(AnglerSetpoints.INTAKE);
      case CLIMB:
        return angler.setAnglerCommand(AnglerSetpoints.CLIMB);
      case EJECT:
        return angler.setAnglerCommand(AnglerSetpoints.FEEDER);
      case UP:
        return angler.setAnglerManualVolts(5.0);
      case DOWN:
        return angler.setAnglerManualVolts(-5.0);
      case IDLE:
        return angler.setAnglerCommand(AnglerSetpoints.IDLE);
      case PODIUM:
        return angler.setAnglerCommand(AnglerSetpoints.PODIUM);
      case SPEAKER:
        return angler.setAnglerCommand(AnglerSetpoints.SPEAKER);
      case FEEDER:
        return angler.setAnglerCommand(AnglerSetpoints.FEEDER);
      case AMP:
        return angler.setAnglerCommand(AnglerSetpoints.AMP);
      default:
        return angler.setAnglerCommand(AnglerSetpoints.IDLE);
    }
  }

  public Command getLauncherCommand(LauncherStates launcherState) {
    switch(launcherState) {
      case OFF:
        return Commands.runOnce(() -> launcher.stopMotors(), this);
      case SPEAKER_SHOT:
        return launcher.setVelocityMPS(LauncherSetpoints.SPEAKER_SHOT);
      case IDLE:
        return launcher.setVelocityMPS(LauncherSetpoints.IDLE);
      case FEEDER:
        return launcher.setVelocityMPS(LauncherSetpoints.FEEDER);
      case REV_AMP:
        return launcher.setVelocityMPS(LauncherSetpoints.AMP);
      case SHOOT_AMP:
        return Commands.runOnce(() -> {
          launcher.setLauncherVelocityMPS(null);
          launcher.setTopLauncherVolts(0);
        }, this);
      case EJECT:
        return launcher.setVelocityMPS(LauncherSetpoints.EJECT);
      case FULL_SPEED:
        return launcher.setVelocityMPS(LauncherSetpoints.FULL_SPEED);
      default:
        return launcher.setVelocityMPS(LauncherSetpoints.IDLE);
    }
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