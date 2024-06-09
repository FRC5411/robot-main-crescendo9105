// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates.ShooterStates;
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
    IDLE(() -> Angler.getLastAnglerPosition()),
    PODIUM(() -> Rotation2d.fromDegrees(34.5)),
    SPEAKER(() -> Rotation2d.fromDegrees(57.0)),
    AMP(() -> Rotation2d.fromDegrees(54.0)),
    FEEDER(() -> Rotation2d.fromDegrees(55.0)),
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
    FEEDER(() -> 23.0, () -> 23.0),
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

  public HashMap<ShooterStates, Command> mapToCommand() {
    HashMap<ShooterStates, Command> shooterCommandMap = new HashMap<>();
    shooterCommandMap.put(
      ShooterStates.OFF, Commands.runOnce(() -> stopMotors(true, true), this));

    shooterCommandMap.put(
      ShooterStates.AIM, setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.SPEAKER_SHOT));

    shooterCommandMap.put(
      ShooterStates.INTAKE, setShooterState(AnglerSetpoints.INTAKE, LauncherSetpoints.OFF));

    shooterCommandMap.put(
      ShooterStates.CLIMB, setShooterState(AnglerSetpoints.CLIMB, LauncherSetpoints.OFF));

    shooterCommandMap.put(
      ShooterStates.EJECT, setShooterState(AnglerSetpoints.CLIMB, LauncherSetpoints.EJECT));

    shooterCommandMap.put(ShooterStates.UP, angler.setVoltageManually(5.0));
    shooterCommandMap.put(ShooterStates.DOWN, angler.setVoltageManually(-5.0));

    shooterCommandMap.put(
      ShooterStates.FIRE, setShooterState(AnglerSetpoints.IDLE, LauncherSetpoints.SPEAKER_SHOT));

    shooterCommandMap.put(
      ShooterStates.IDLE,
      Commands.runOnce(
        () -> {
          setMotors(null, LauncherSetpoints.OFF);
          angler.setAnglerVolts(0.0);
        }, this));

    shooterCommandMap.put(
      ShooterStates.PODIUM, setShooterState(AnglerSetpoints.PODIUM, LauncherSetpoints.SPEAKER_SHOT));

    shooterCommandMap.put(
        ShooterStates.SPEAKER, setShooterState(AnglerSetpoints.SPEAKER, LauncherSetpoints.SPEAKER_SHOT));

    shooterCommandMap.put(
        ShooterStates.FEEDER, setShooterState(AnglerSetpoints.FEEDER, LauncherSetpoints.FEEDER));
        
    shooterCommandMap.put(
        ShooterStates.REV_AMP, setShooterState(AnglerSetpoints.AMP, LauncherSetpoints.AMP));

    shooterCommandMap.put(
      ShooterStates.SHOOT_AMP, Commands.runOnce(() -> {
        setMotors(angler.getCurrentSetpoint(), null);
        launcher.setTopLauncherVolts(0);
      }, this));

    return shooterCommandMap;
  }

  public Command setShooterState(
      AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return angler.setAnglerSetpointCommand(anglerState).alongWith(launcher.setVelocityMPS(launcherState));
  }

  public void setMotors(AnglerSetpoints anglerGoal, LauncherSetpoints launcherGoal) {
    angler.setAnglerSetpoint(anglerGoal);
    launcher.setLauncherVelocityMPS(launcherGoal);
  }

  public void stopMotors(boolean stopLauncher, boolean stopAngler) {
    if(stopLauncher) launcher.stopMotors();
    if(stopAngler) angler.stopMotor();
  }

  public boolean atAllSetpoints() {
    return angler.atAnglerSetpoint() && launcher.atFlywheelSetpoints();
  }

  public Angler getAngler() {
    return angler;
  }

  public Launcher getLauncher() {
    return launcher;
  }
}