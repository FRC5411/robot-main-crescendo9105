package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.RobotStates.YoshiStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.yoshivator.Yoshivator;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {
  private Shooter robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Yoshivator robotYoshi;
  private Climb robotClimb;

  private ShooterStates shooterState;
  private IntakeStates intakeState;
  private IndexerStates indexerState;
  private YoshiStates yoshiState;
  private ClimbStates climbState;

  public StateMachine(
      Shooter robotShooter,
      Intake robotIntake,
      Indexer robotIndexer,
      Yoshivator robotYoshi,
      Climb robotClimb) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotIndexer = robotIndexer;
    this.robotYoshi = robotYoshi;
    this.robotClimb = robotClimb;

    shooterState = ShooterStates.AIM;
    intakeState = IntakeStates.OFF;
    indexerState = IndexerStates.OFF;
    yoshiState = YoshiStates.IDLE;
    climbState = ClimbStates.OFF;
  }

  public Command getShooterCommand(ShooterStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  shooterState = state;
                  robotShooter.mapToCommand(shooterState).schedule();
                }))
        .withName("StateMachince/ShooterCommand/" + shooterState);
  }

  public Command getIntakeCommand(IntakeStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  intakeState = state;
                  robotIntake.mapToCommand(intakeState).schedule();
                }))
        .withName("StateMachince/IntakeCommand/" + intakeState);
  }

  public Command getIndexerCommand(IndexerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  indexerState = state;
                  robotIndexer.mapToCommand(indexerState).schedule();
                }))
        .withName("StateMachince/IndexerCommand/" + indexerState);
  }

  public Command getYoshiCommand(YoshiStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  yoshiState = state;
                  robotYoshi.mapToCommand(yoshiState).schedule();
                }))
        .withName("StateMachince/YoshiCommand/" + yoshiState);
  }

  public Command getClimbCommand(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  climbState = state;
                  robotClimb.mapToCommand(climbState).schedule();
                }))
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

  @AutoLogOutput(key = "StateMachine/YoshiState")
  public YoshiStates getYoshiState() {
    return yoshiState;
  }

  @AutoLogOutput(key = "StateMachine/ClimbState")
  public ClimbStates getClimbState() {
    return climbState;
  }
}
