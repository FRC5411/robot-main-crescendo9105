package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.RobotStates.YoshiStates;

public class DriverCommands {
  private static StateMachine stateMachine;

  public static void setStateMachine(StateMachine machine) {
    stateMachine = machine;
  }

  /** Intake and amp commands */
  public static Command intakeNote() {
    return new ParallelCommandGroup(
        stateMachine.getShooterCommand(ShooterStates.INTAKE),
        stateMachine.getIntakeCommand(IntakeStates.INTAKE),
        stateMachine.getIndexerCommand(IndexerStates.STOW));
  }

  public static Command yoshiIntakeNote() {
    return intakeNote().alongWith(stateMachine.getYoshiCommand(YoshiStates.GROUND));
  }

  public static Command outtakeNote() {
    return new ParallelCommandGroup(
        stateMachine.getYoshiCommand(YoshiStates.IDLE),
        stateMachine.getIntakeCommand(IntakeStates.OUTTAKE),
        stateMachine.getIndexerCommand(IndexerStates.OUTDEX),
        stateMachine.getShooterCommand(ShooterStates.INTAKE));
  }

  public static Command stopTakeNote() {
    return new ParallelCommandGroup(
        stateMachine.getYoshiCommand(YoshiStates.IDLE),
        stateMachine.getIntakeCommand(IntakeStates.OFF),
        stateMachine.getIndexerCommand(IndexerStates.OFF),
        stateMachine.getShooterCommand(ShooterStates.AIM));
  }

  public static Command scoreAmp() {
    return stateMachine.getYoshiCommand(YoshiStates.AMP);
  }

  /** Shooting logic */
  public static Command prepareNoteShot() {
    return stateMachine.getShooterCommand(ShooterStates.AIM);
  }

  public static Command shootNote() {
    return new ParallelCommandGroup(
        stateMachine.getIndexerCommand(IndexerStates.INDEX),
        new ConditionalCommand(
            stateMachine.getShooterCommand(ShooterStates.FIRE),
            new InstantCommand(),
            () -> stateMachine.getShooterState() != ShooterStates.AIM));
  }

  public static Command stopShooting() {
    return new ParallelCommandGroup(
        stateMachine.getIndexerCommand(IndexerStates.OFF),
        stateMachine.getShooterCommand(ShooterStates.AIM));
  }

  public static Command moveAnglerUpManual() {
    return stateMachine.getShooterCommand(ShooterStates.UP);
  }

  public static Command moveAnglerDownManual() {
    return stateMachine.getShooterCommand(ShooterStates.DOWN);
  }

  public static Command shooterToIdle() {
    return stateMachine.getShooterCommand(ShooterStates.IDLE);
  }

  /** Climb logic */
  public static Command climbChain() {
    return stateMachine.getClimbCommand(ClimbStates.MOVE_BOTH);
  }

  public static Command adjustLeftClimb() {
    return stateMachine.getClimbCommand(ClimbStates.MOVE_LEFT);
  }

  public static Command adjustRight() {
    return stateMachine.getClimbCommand(ClimbStates.MOVE_RIGHT);
  }

  public static Command invertClimb() {
    return stateMachine.getClimbCommand(ClimbStates.INVERT);
  }

  public static Command stopClimb() {
    return stateMachine.getClimbCommand(ClimbStates.OFF);
  }
}
