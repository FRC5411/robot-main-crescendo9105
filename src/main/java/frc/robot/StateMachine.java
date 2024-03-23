package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbVoltSetpoints;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoint;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.angler.Angler.AnglerSetpoints;
import frc.robot.subsystems.shooter.angler.Angler.LauncherSetpoints;
import frc.robot.utils.commands.CommandUtils;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {
  private Angler robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Climb robotClimb;

  private ShooterStates shooterState;
  private IntakeStates intakeState;
  private IndexerStates indexerState;
  private ClimbStates climbState;

  private SelectCommand<ShooterStates> shooterCommands;
  private SelectCommand<IntakeStates> intakeCommands;
  private SelectCommand<IndexerStates> indexerCommands;
  private SelectCommand<ClimbStates> climbCommands;

  public StateMachine(
      Angler robotShooter, Intake robotIntake, Indexer robotIndexer, Climb robotClimb) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotIndexer = robotIndexer;
    this.robotClimb = robotClimb;

    shooterState = ShooterStates.IDLE;
    intakeState = IntakeStates.OFF;
    indexerState = IndexerStates.OFF;
    climbState = ClimbStates.OFF;

    shooterCommands =
        new SelectCommand<ShooterStates>(this.robotShooter.mapToCommand(), () -> shooterState);

    intakeCommands =
        new SelectCommand<IntakeStates>(this.robotIntake.mapToCommand(), () -> intakeState);

    indexerCommands =
        new SelectCommand<IndexerStates>(this.robotIndexer.mapToCommand(), () -> indexerState);

    climbCommands =
        new SelectCommand<ClimbStates>(this.robotClimb.mapToCommand(), () -> climbState);

    robotShooter.setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.IDLE);
    robotIntake.setCurrentSetpoint(IntakeSetpoint.OFF);
    robotIndexer.setCurrentSetpoint(IndexerSetpoint.OFF);
    robotClimb.setManualVolts(ClimbVoltSetpoints.OFF);
  }

  public Command setShooterState(ShooterStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> shooterState = state),
            CommandUtils.copyCommand(shooterCommands))
        .withName("StateMachince/ShooterCommand/" + shooterState);
  }

  public Command setIntakeState(IntakeStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeState = state), CommandUtils.copyCommand(intakeCommands))
        .withName("StateMachince/IntakeCommand/" + intakeState);
  }

  public Command setIndexerState(IndexerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> indexerState = state),
            CommandUtils.copyCommand(indexerCommands))
        .withName("StateMachince/IndexerCommand/" + indexerState);
  }

  public Command setClimbState(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> climbState = state), CommandUtils.copyCommand(climbCommands))
        .withName("StateMachince/ClimbCommand/" + climbState);
  }



  @AutoLogOutput(key = "StateMachine/ShooterState")
  public ShooterStates getShooterState() {
    return shooterState;
  }

  @AutoLogOutput(key = "StateMachine/IntakeState")
  public IntakeStates getIntakeState() {
    return intakeState;
  }

  @AutoLogOutput(key = "StateMachine/IndexerState")
  public IndexerStates getIndexerState() {
    return indexerState;
  }

  @AutoLogOutput(key = "StateMachine/ClimbState")
  public ClimbStates getClimbState() {
    return climbState;
  }
}
