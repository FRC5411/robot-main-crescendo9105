// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.ClimbCommands.ClimbLeftDirection;
import frc.robot.commands.ClimbCommands.ClimbRightDirection;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.AnglerDirection;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.TargetingSystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** State-machine for the Launcher, Angler, & Climb */
public class Superstructure extends SubsystemBase {
  private Drive robotDrive;
  private Shooter robotShooter;
  private Climb robotClimb;

  private TargetingSystem robotTargetingSystem;

  private SuperstructureState currentState = SuperstructureState.IDLE;
  private BooleanSupplier currentClimbDirectionIn = () -> false;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, Shooter shooter, Climb climb, TargetingSystem targetingSystem) {
    robotDrive = drive;
    robotShooter = shooter;
    robotClimb = climb;

    robotTargetingSystem = targetingSystem;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/CurrentState", currentState);
  }

  /** Returns a command based on the current robot state */
  public Command getCommand(SuperstructureState state) {
    return Commands.runOnce(
            () -> {
              currentState = state;
            },
            this)
        .andThen(
            switch (state) {
              case IDLE -> ShooterCommands.stopShooter(robotShooter, true, true)
                  .alongWith(ClimbCommands.stopClimb(robotClimb));
              case INTAKING -> ShooterCommands.runAll(
                      robotShooter,
                      AnglerPositions.INTAKING.getPosition(),
                      LauncherSpeeds.IDLE.getSpeedsMPS())
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_IDLE.getPosition(),
                          ClimbPositions.RIGHT_IDLE.getPosition()));
              case PREPARING_SHOT -> ShooterCommands.automaticTarget(
                      robotShooter, robotTargetingSystem, () -> robotDrive.getPosition())
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_IDLE.getPosition(),
                          ClimbPositions.RIGHT_IDLE.getPosition()));
              case MANUAL_ANGLER_UP -> ShooterCommands.runAnglerManual(
                      robotShooter, AnglerDirection.UP)
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_IDLE.getPosition(),
                          ClimbPositions.RIGHT_IDLE.getPosition()));
              case MANUAL_ANGLER_DOWN -> ShooterCommands.runAnglerManual(
                      robotShooter, AnglerDirection.DOWN)
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_IDLE.getPosition(),
                          ClimbPositions.RIGHT_IDLE.getPosition()));
              case MANUAL_LAUNCHER -> ShooterCommands.runLauncher(
                  robotShooter, LauncherSpeeds.SETPOINT_SPEED.getSpeedsMPS());
              case PREPARING_CLIMB -> ShooterCommands.runAll(
                      robotShooter,
                      AnglerPositions.CLIMB.getPosition(),
                      LauncherSpeeds.IDLE.getSpeedsMPS())
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_PREPARED.getPosition(),
                          ClimbPositions.RIGHT_PREPARED.getPosition()));
              case CLIMBING -> ShooterCommands.runAll(
                      robotShooter,
                      AnglerPositions.CLIMB.getPosition(),
                      LauncherSpeeds.IDLE.getSpeedsMPS())
                  .alongWith(
                      ClimbCommands.runClimb(
                          robotClimb,
                          ClimbPositions.LEFT_HANGING.getPosition(),
                          ClimbPositions.RIGHT_HANGING.getPosition()));
              case MANUAL_CLIMB_LEFT -> ShooterCommands.stopShooter(robotShooter, true, true)
                  .alongWith(
                      (currentClimbDirectionIn.getAsBoolean())
                          ? ClimbCommands.runLeftClimbManual(robotClimb, ClimbLeftDirection.IN)
                          : ClimbCommands.runLeftClimbManual(robotClimb, ClimbLeftDirection.OUT));
              case MANUAL_CLIMB_RIGHT -> ShooterCommands.stopShooter(robotShooter, true, true)
                  .alongWith(
                      (currentClimbDirectionIn.getAsBoolean())
                          ? ClimbCommands.runRightClimbManual(robotClimb, ClimbRightDirection.IN)
                          : ClimbCommands.runRightClimbManual(robotClimb, ClimbRightDirection.OUT));
              case DIAGNOSTIC -> null;
            });
  }

  /** Predefined states of the superstructure */
  public static enum SuperstructureState {
    /** All motors are idle (stopped) */
    IDLE,
    /** Move the Shooter-Angler to a position to safely intake piece - All other motors are idle */
    INTAKING,
    /**
     * Move the Shooter-Angler to to a position dictated by the TargetingSystem - Spin the
     * Shooter-Launcher to speeds of 38mps - Climb is idle
     */
    PREPARING_SHOT,
    /** Move the Shooter-Angler using manual up movements - Maintain all other states */
    MANUAL_ANGLER_UP,
    /** Move the Shooter-Angler using manual down movements - Maintain all other states */
    MANUAL_ANGLER_DOWN,
    /** Spin Shooter-Launcher to 38mps or 0mps - Maintain all other states */
    MANUAL_LAUNCHER,
    /**
     * Move the Shooter-Angler to the down-most position - Run the Shooter-Laucnher to 0mps - Move
     * the Climb arms to the outward position
     */
    PREPARING_CLIMB,
    /**
     * Maintain Shooter-Angler position - Maintain Shooter-Launcher speed - Run the Climb to the
     * compressed setpoint to hang to the chain
     */
    CLIMBING,
    /** Shooter maintains current state - Climb left arm move in or out based on manual controls */
    MANUAL_CLIMB_LEFT,
    /** Shooter maintains current state - Climb right arm move in or out based on manual controls */
    MANUAL_CLIMB_RIGHT,
    /** Run the superstructure's diagnostic commands as needed */
    DIAGNOSTIC
  }

  /** Fixed setpoints for the Angler */
  public static enum AnglerPositions {
    CLIMB(Rotation2d.fromDegrees(25.0)),
    INTAKING(Rotation2d.fromDegrees(45.0)),
    FIXED(Rotation2d.fromDegrees(55.0));

    private Rotation2d desiredPosition = new Rotation2d();

    /** Set the angler arms to a fixed position */
    AnglerPositions(Rotation2d position) {
      this.desiredPosition = position;
    }

    /** Returns the desired angler arm position */
    public Rotation2d getPosition() {
      return this.desiredPosition;
    }
  }

  /** Fixed speeds in MPS for the Launcher */
  public static enum LauncherSpeeds {
    IDLE(10.0),
    SETPOINT_SPEED(38.0),
    COURT_SHOT(42.0);

    private double desiredSpeeds;

    /** Set the launcher to a desired speed */
    LauncherSpeeds(double speeds) {
      this.desiredSpeeds = speeds;
    }

    /** Returns the desired launcher flywheel speeds */
    public double getSpeedsMPS() {
      return this.desiredSpeeds;
    }
  }

  /** Fixed setpoints for the Climb */
  public static enum ClimbPositions {
    LEFT_IDLE(Rotation2d.fromDegrees(180.0)),
    RIGHT_IDLE(Rotation2d.fromDegrees(180.0)),
    LEFT_PREPARED(Rotation2d.fromDegrees(0.0)),
    RIGHT_PREPARED(Rotation2d.fromDegrees(0.0)),
    LEFT_HANGING(Rotation2d.fromDegrees(-170.0)),
    RIGHT_HANGING(Rotation2d.fromDegrees(-160.0));

    private Rotation2d desiredPosition = new Rotation2d();

    /** Set the climb arms to a fixed position */
    ClimbPositions(Rotation2d position) {
      this.desiredPosition = position;
    }

    /** Returns the desired climb arm position */
    public Rotation2d getPosition() {
      return this.desiredPosition;
    }
  }
}
