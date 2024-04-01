package frc.robot.managers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.managers.RobotStates.AnglerStates;
import frc.robot.managers.RobotStates.ClimbStates;
import frc.robot.managers.RobotStates.IndexerStates;
import frc.robot.managers.RobotStates.IntakeStates;
import frc.robot.managers.RobotStates.LauncherStates;
import frc.robot.managers.RobotStates.YoshiStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbVoltSetpoints;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.AnglerSetpoints;
import frc.robot.subsystems.shooter.Shooter.LauncherSetpoints;
import frc.robot.utils.commands.DynamicCommand;

import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.subsystems.yoshivator.Yoshivator.YoshivatorSetpoints;

public class StateMachine {
  private Shooter robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Climb robotClimb;
  private Yoshivator robotYoshi;

  private LauncherStates launcherState;
  private AnglerStates anglerState;
  private IntakeStates intakeState;
  private IndexerStates indexerState;
  private ClimbStates climbState;
  private YoshiStates yoshiState;

  private DynamicCommand launcherCommands;
  private DynamicCommand anglerCommands;
  private DynamicCommand intakeCommands;
  private DynamicCommand indexerCommands;
  private DynamicCommand climbCommands;
  private DynamicCommand yoshiCommands;

  public StateMachine(
      Shooter robotShooter, Intake robotIntake, Indexer robotIndexer, Climb robotClimb, Yoshivator robotYoshi) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotIndexer = robotIndexer;
    this.robotClimb = robotClimb;
    this.robotYoshi = robotYoshi;

    launcherState = LauncherStates.IDLE;
    anglerState = AnglerStates.IDLE;
    intakeState = IntakeStates.OFF;
    indexerState = IndexerStates.OFF;
    climbState = ClimbStates.OFF;
    yoshiState = YoshiStates.IDLE;

    launcherCommands =
        new DynamicCommand(
            () -> this.robotShooter.getLauncherCommand(getLauncherState()), this.robotShooter.getLauncher());
    anglerCommands =
        new DynamicCommand(
            () -> this.robotShooter.getAnglerCommand(getAnglerState()), this.robotShooter.getAngler());
    
    intakeCommands =
        new DynamicCommand(
            () -> this.robotIntake.getIntakeCommand(getIntakeState()), this.robotIntake);

    indexerCommands =
        new DynamicCommand(
            () -> this.robotIndexer.getIndexerCommand(getIndexerState()), this.robotIndexer);
    
    climbCommands =
        new DynamicCommand(
            () -> this.robotClimb.getClimbCommand(getClimbState()), this.robotClimb);

    yoshiCommands =
        new DynamicCommand(
            () -> this.robotYoshi.getYoshiCommand(getYoshiState()), this.robotYoshi);

    robotShooter.setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.IDLE);
    robotIntake.setCurrentSetpoint(IntakeSetpoint.OFF);
    robotIndexer.setCurrentSetpoint(IndexerSetpoint.OFF);
    robotClimb.runVolts(ClimbVoltSetpoints.OFF);
    robotYoshi.setYoshiSetpoint(YoshivatorSetpoints.IDLE);
  }

  public Command getShooterCommand(LauncherStates launcherState, AnglerStates anglerState) {
    return new ParallelCommandGroup(
      getLauncherCommand(launcherState), 
      getAnglerCommand(anglerState)
    );
  }

  public Command getLauncherCommand(LauncherStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> launcherState = state),
            launcherCommands.copy())
        .withName("StateMachince/ShooterCommand/" + launcherState);
  }

  public Command getAnglerCommand(AnglerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> anglerState = state), 
            anglerCommands.copy())
        .withName("StateMachince/AnglerCommand/" + anglerState);
  }

  public Command getIntakeCommand(IntakeStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeState = state), 
            intakeCommands.copy())
        .withName("StateMachince/IntakeCommand/" + intakeState);
  }

  public Command getIndexerCommand(IndexerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> indexerState = state),
            indexerCommands.copy())
        .withName("StateMachince/IndexerCommand/" + indexerState);
  }

  public Command getClimbCommand(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> climbState = state), 
            climbCommands.copy())
        .withName("StateMachince/ClimbCommand/" + climbState);
  }

  public Command getYoshiCommand(YoshiStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> yoshiState = state), 
            yoshiCommands.copy())
        .withName("StateMachince/YoshiCommand/" + yoshiState);
  }

  public Command intakeNote() {
    return new ParallelCommandGroup(
        getShooterCommand(LauncherStates.OFF, AnglerStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getIndexerCommand(IndexerStates.STOW),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command yoshiIntakeNote() {
    return new ParallelCommandGroup(
        getShooterCommand(LauncherStates.OFF, AnglerStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getIndexerCommand(IndexerStates.STOW),
        getClimbCommand(ClimbStates.IDLE),
        getYoshiCommand(YoshiStates.GROUND_INTAKE));
  }

  public Command outtakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OUTTAKE),
        getIndexerCommand(IndexerStates.OUTDEX),
        getShooterCommand(LauncherStates.OFF, AnglerStates.INTAKE));
  }

  public Command stopTakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OFF),
        getIndexerCommand(IndexerStates.OFF),
        getShooterCommand(LauncherStates.IDLE, AnglerStates.IDLE),
        getYoshiCommand(YoshiStates.IDLE));
  }

  public Command podiumShot() {
    return getShooterCommand(LauncherStates.SPEAKER_SHOT, AnglerStates.PODIUM);
  }

  public Command speakerShot() {
    return getShooterCommand(LauncherStates.SPEAKER_SHOT, AnglerStates.SPEAKER);
  }

  public Command feedShot() {
    return getShooterCommand(LauncherStates.FEEDER, AnglerStates.FEEDER);
  }

  public Command climbToAmp() {
    return getClimbCommand(ClimbStates.AMP);
  }

  public Command prepareNoteShot() {
    return getShooterCommand(LauncherStates.SPEAKER_SHOT, AnglerStates.AIM);
  }

  public Command shootNote() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.INDEX));
  }

  public Command revUp() {
    return getLauncherCommand(LauncherStates.SPEAKER_SHOT);
  }

  public Command runIndexer() {
    return getIndexerCommand(IndexerStates.INDEX);
  }

  public Command stopShooting() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.OFF), 
        getShooterCommand(
          LauncherStates.IDLE, 
          AnglerStates.IDLE));
  }

  public Command moveAnglerUpManual() {
    return getAnglerCommand(AnglerStates.UP);
  }

  public Command moveAnglerDownManual() {
    return getAnglerCommand(AnglerStates.DOWN);
  }

  public Command shooterToIdle() {
    return getShooterCommand(LauncherStates.IDLE, AnglerStates.IDLE);
  }

  public Command climbDown() {
    return new ParallelCommandGroup(
      getClimbCommand(ClimbStates.MOVE_BOTH_UP), 
      getShooterCommand(LauncherStates.OFF, AnglerStates.CLIMB));
  }

  public Command climbUp() {
    return new ParallelCommandGroup(
      getClimbCommand(ClimbStates.MOVE_BOTH_DOWN), 
      getShooterCommand(LauncherStates.OFF, AnglerStates.CLIMB));
  }

  public Command stopClimb() {
    return new ParallelCommandGroup(
      getClimbCommand(ClimbStates.OFF), 
      getShooterCommand(LauncherStates.OFF, AnglerStates.CLIMB));
  }

  public Command eject() {
    return getShooterCommand(LauncherStates.EJECT, AnglerStates.EJECT);
  }

  public Command revAmp() {
    return getShooterCommand(LauncherStates.REV_AMP, AnglerStates.AMP);
  }

  public Command scoreAmp() {
    return getShooterCommand(LauncherStates.SHOOT_AMP, AnglerStates.AMP).andThen(getIndexerCommand(IndexerStates.AMP));
  }

  @AutoLogOutput(key = "StateMachine/LauncherState")
  public LauncherStates getLauncherState() {
    return launcherState;
  }

  @AutoLogOutput(key = "StateMachine/AnglerState")
  public AnglerStates getAnglerState() {
    return anglerState;
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

  @AutoLogOutput(key = "StateMachine/YoshiState")
  public YoshiStates getYoshiState() {
    return yoshiState;
  }
}
