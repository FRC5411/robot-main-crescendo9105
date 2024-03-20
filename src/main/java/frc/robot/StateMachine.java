package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {
  private Shooter robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Climb robotClimb;

  private ShooterStates shooterState;
  private IntakeStates intakeState;
  private IndexerStates indexerState;
  private ClimbStates climbState;

  public StateMachine(
      Shooter robotShooter, Intake robotIntake, Indexer robotIndexer, Climb robotClimb) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotIndexer = robotIndexer;
    this.robotClimb = robotClimb;

    shooterState = ShooterStates.OFF;
    intakeState = IntakeStates.OFF;
    indexerState = IndexerStates.OFF;
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

  public Command getClimbCommand(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  climbState = state;
                  robotClimb.mapToCommand(climbState).schedule();
                }))
        .withName("StateMachince/ClimbCommand/" + climbState);
  }

  public Command intakeNote() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getIndexerCommand(IndexerStates.STOW),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command podiumShot() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.PODIUM), getClimbCommand(ClimbStates.IDLE));
  }

  public Command speakerShot() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.SPEAKER), getClimbCommand(ClimbStates.IDLE));
  }

  public Command feedShot() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.FEEDER), getClimbCommand(ClimbStates.IDLE));
  }

  public Command outtakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OUTTAKE),
        getIndexerCommand(IndexerStates.OUTDEX),
        getShooterCommand(ShooterStates.INTAKE),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command stopTakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OFF),
        getIndexerCommand(IndexerStates.OFF),
        getShooterCommand(ShooterStates.OFF),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command climbToAmp() {
    return getClimbCommand(ClimbStates.AMP);
  }

  public Command prepareNoteShot() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.AIM), getClimbCommand(ClimbStates.IDLE));
  }

  public Command shootNote() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.INDEX),
        getShooterCommand(ShooterStates.FIRE),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command revUp() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.FIRE), getClimbCommand(ClimbStates.IDLE));
  }

  public Command ejectSlow() {
    return getShooterCommand(ShooterStates.EJECT);
  }

  public Command runIndexer() {
    return getIndexerCommand(IndexerStates.INDEX);
  }

  public Command stopShooting() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.OFF),
        getShooterCommand(ShooterStates.IDLE),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command moveAnglerUpManual() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.UP), getClimbCommand(ClimbStates.IDLE));
  }

  public Command moveAnglerDownManual() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.DOWN), getClimbCommand(ClimbStates.IDLE));
  }

  public Command shooterToIdle() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.IDLE), getClimbCommand(ClimbStates.IDLE));
  }

  public Command climbDown() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_UP), getShooterCommand(ShooterStates.CLIMB));
  }

  public Command climbUp() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_DOWN), getShooterCommand(ShooterStates.CLIMB));
  }

  public Command stopClimb() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.OFF), getShooterCommand(ShooterStates.CLIMB));
  }

  public Command eject() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.EJECT));
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
