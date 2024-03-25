// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates.AnglerStates;
import frc.robot.RobotStates.LauncherStates;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.launcher.Launcher;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Shooter subsystem */
public class Shooter extends SubsystemBase {
  private static LoggedTunableNumber angleTunableNumber = new LoggedTunableNumber("Shooter/AngleDebuggingDegrees", 0.0);
   
  public static enum AnglerSetpoints {
    AIM(() -> TargetingSystem.getInstance().getLaunchMapAngle()),
    CLIMB(() -> Rotation2d.fromDegrees(25.0)),
    INTAKE(() -> Rotation2d.fromDegrees(45.0)),
    IDLE(() -> anglerPosition),
    PODIUM(() -> Rotation2d.fromDegrees(34.5)),
    SPEAKER(() -> Rotation2d.fromDegrees(57)),
    AMP(() -> Rotation2d.fromDegrees(54.0)),
    FEEDER(() -> Rotation2d.fromDegrees(50.0)),
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
    FEEDER(() -> 30.0, () -> 30.0),
    // 12.5: 7 / 9
    // 12.65: 0
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

  // public HashMap<ShooterStates, Command> mapToCommand() {
  //   HashMap<ShooterStates, Command> shooterCommandMap = new HashMap<>();
  //   shooterCommandMap.put(ShooterStates.OFF, Commands.runOnce(() -> stopMotors(true, true), this));
  //   shooterCommandMap.put(
  //       ShooterStates.AIM,
  //       setShooterStateWithEnd(AnglerSetpoints.AIM, LauncherSetpoints.SPEAKER_SHOT));
  //   shooterCommandMap.put(
  //       ShooterStates.INTAKE, setShooterStateWithEnd(AnglerSetpoints.INTAKE, LauncherSetpoints.OFF));
  //   shooterCommandMap.put(
  //       ShooterStates.CLIMB, setShooterStateWithEnd(AnglerSetpoints.CLIMB, LauncherSetpoints.OFF));
  //   shooterCommandMap.put(
  //       ShooterStates.EJECT, setShooterStateWithEnd(AnglerSetpoints.CLIMB, LauncherSetpoints.EJECT));
  //   shooterCommandMap.put(ShooterStates.UP, setAnglerManual(5.0));
  //   shooterCommandMap.put(ShooterStates.DOWN, setAnglerManual(-5.0));
  //   shooterCommandMap.put(
  //       ShooterStates.FIRE,
  //       Commands.runOnce(() -> launcher.setLauncherVelocityMPS(LauncherSetpoints.SPEAKER_SHOT), this));
  //   shooterCommandMap.put(
  //       ShooterStates.IDLE,
  //       Commands.runOnce(
  //           () -> {
  //             setMotors(null, LauncherSetpoints.OFF);
  //             angler.setAnglerVolts(0.0);
  //           },
  //           this));
  //   shooterCommandMap.put(
  //       ShooterStates.PODIUM,
  //       setShooterStateWithEnd(AnglerSetpoints.PODIUM, LauncherSetpoints.SPEAKER_SHOT));
  //   shooterCommandMap.put(
  //       ShooterStates.SPEAKER,
  //       setShooterStateWithEnd(AnglerSetpoints.SPEAKER, LauncherSetpoints.SPEAKER_SHOT));
  //   shooterCommandMap.put(
  //       ShooterStates.FEEDER,
  //       setShooterStateWithEnd(AnglerSetpoints.FEEDER, LauncherSetpoints.FULL_SPEED));
  //   shooterCommandMap.put(
  //       ShooterStates.REV_AMP, setShooterState(AnglerSetpoints.AMP, LauncherSetpoints.AMP));

  //   shooterCommandMap.put(
  //     ShooterStates.SHOOT_AMP, Commands.runOnce(() -> {
  //       setMotors(angler.getCurrentSetpoint(), null);
  //       launcher.setTopLauncherVolts(0);
  //     }, this));
    
  //   shooterCommandMap.put(ShooterStates.AIM_AUTON, setShooterStateWithEnd(AnglerSetpoints.AIM, LauncherSetpoints.SPEAKER_SHOT));

  //   return shooterCommandMap;
  // }

  public HashMap<AnglerStates, Command> mapToAnglerCommand() {
    HashMap<AnglerStates, Command> anglerCommandMap = new HashMap<>();
    anglerCommandMap.put(AnglerStates.OFF, Commands.runOnce(() -> angler.stopMotor(), angler));
    anglerCommandMap.put(AnglerStates.UP, angler.setAnglerManual(5.0));
    anglerCommandMap.put(AnglerStates.DOWN, angler.setAnglerManual(-5.0));
    anglerCommandMap.put(AnglerStates.AMP, angler.setAnglerCommandWithEnd(AnglerSetpoints.AMP));
    anglerCommandMap.put(AnglerStates.AIM, angler.setAnglerCommandWithEnd(AnglerSetpoints.AIM));
    anglerCommandMap.put(AnglerStates.AIM_IDLE, angler.setAnglerCommandWithoutEnd(AnglerSetpoints.AIM));
    anglerCommandMap.put(AnglerStates.FEEDER, angler.setAnglerCommandWithEnd(AnglerSetpoints.FEEDER));
    anglerCommandMap.put(AnglerStates.IDLE, angler.setAnglerCommandWithEnd(AnglerSetpoints.IDLE));
    anglerCommandMap.put(AnglerStates.INTAKE, angler.setAnglerCommandWithEnd(AnglerSetpoints.INTAKE));
    anglerCommandMap.put(AnglerStates.PODIUM, angler.setAnglerCommandWithEnd(AnglerSetpoints.PODIUM));
    anglerCommandMap.put(AnglerStates.SPEAKER, angler.setAnglerCommandWithEnd(AnglerSetpoints.SPEAKER));
    anglerCommandMap.put(AnglerStates.EJECT, angler.setAnglerCommandWithEnd(AnglerSetpoints.FEEDER));
    anglerCommandMap.put(AnglerStates.CLIMB, angler.setAnglerCommandWithEnd(AnglerSetpoints.CLIMB));

    return anglerCommandMap;
  }

  public HashMap<LauncherStates, Command> mapToLauncherCommand() {
    HashMap<LauncherStates, Command> flywheelCommandMap = new HashMap<>();
    flywheelCommandMap.put(LauncherStates.OFF, Commands.runOnce(() -> launcher.stopMotors(), launcher));
    flywheelCommandMap.put(LauncherStates.IDLE, launcher.setVelocityMPS(LauncherSetpoints.IDLE));
    flywheelCommandMap.put(LauncherStates.REV_AMP, launcher.setVelocityMPS(LauncherSetpoints.AMP));
    flywheelCommandMap.put(LauncherStates.SCORE_AMP, Commands.runOnce(() -> {
      launcher.setVelocityMPS(null);
      launcher.setTopLauncherVolts(0.0);
    }, launcher));
    flywheelCommandMap.put(LauncherStates.EJECT,  launcher.setVelocityMPS(LauncherSetpoints.EJECT));
    flywheelCommandMap.put(LauncherStates.FEEDER, launcher.setVelocityMPS(LauncherSetpoints.FEEDER));
    flywheelCommandMap.put(LauncherStates.SHOOT, launcher.setVelocityMPS(LauncherSetpoints.SPEAKER_SHOT));
    flywheelCommandMap.put(LauncherStates.FULL_SPEED, launcher.setVelocityMPS(LauncherSetpoints.FULL_SPEED));

    return flywheelCommandMap;
  }

  public Command setFlywheelState(LauncherSetpoints setpoint) {
    return launcher.setVelocityMPS(setpoint);
  }

  public Command setShooterStateWithEnd(
      AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return angler.setAnglerCommandWithEnd(anglerState);
  }

  public Command setShooterState(
      AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return angler.setAnglerCommandWithoutEnd(anglerState);
  }

  public void setMotors(AnglerSetpoints anglerGoal, LauncherSetpoints launcherGoal) {
    angler.setAnglerPosition(anglerGoal);
    launcher.setLauncherVelocityMPS(launcherGoal);
  }

  public void stopMotors(boolean stopLauncher, boolean stopAngler) {
    if(stopLauncher)launcher.stopMotors();
    if(stopAngler) angler.stopMotor();
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