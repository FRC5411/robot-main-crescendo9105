package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStates.AnglerStates;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.LauncherStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbVoltSetpoints;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.AnglerSetpoints;
import frc.robot.subsystems.shooter.Shooter.LauncherSetpoints;
import frc.robot.utils.commands.CommandUtils;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {
  private Shooter robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Climb robotClimb;

  private LauncherStates launcherState;
  private AnglerStates anglerState;
  private IntakeStates intakeState;
  private IndexerStates indexerState;
  private ClimbStates climbState;

  private SelectCommand<LauncherStates> launcherCommands;
  private SelectCommand<AnglerStates> anglerCommands;
  private SelectCommand<IntakeStates> intakeCommands;
  private SelectCommand<IndexerStates> indexerCommands;
  private SelectCommand<ClimbStates> climbCommands;

  public StateMachine(
      Shooter robotShooter, Intake robotIntake, Indexer robotIndexer, Climb robotClimb) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotIndexer = robotIndexer;
    this.robotClimb = robotClimb;

    launcherState = LauncherStates.IDLE;
    anglerState = AnglerStates.IDLE;
    intakeState = IntakeStates.OFF;
    indexerState = IndexerStates.OFF;
    climbState = ClimbStates.OFF;

    launcherCommands =
        new SelectCommand<LauncherStates>(this.robotShooter.mapToLauncherCommand(), () -> launcherState);

    anglerCommands = 
        new SelectCommand<AnglerStates>(this.robotShooter.mapToAnglerCommand(), () -> anglerState);

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

  public Command getShooterCommand(LauncherStates launchState, AnglerStates anglerState) {
    return new ParallelCommandGroup(
      getLauncherCommand(launchState),
      getAnglerCommand(anglerState)
    );
  }

  public Command getLauncherCommand(LauncherStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> launcherState = state),
            CommandUtils.copyCommand(launcherCommands))
        .withName("StateMachince/ShooterCommand/" + launcherState);
  }

  public Command getAnglerCommand(AnglerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> anglerState = state),
            CommandUtils.copyCommand(anglerCommands))
        .withName("StateMachince/ShooterCommand/" + anglerState);
  }

  public Command getIntakeCommand(IntakeStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeState = state), CommandUtils.copyCommand(intakeCommands))
        .withName("StateMachince/IntakeCommand/" + intakeState);
  }

  public Command getIndexerCommand(IndexerStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> indexerState = state),
            CommandUtils.copyCommand(indexerCommands))
        .withName("StateMachince/IndexerCommand/" + indexerState);
  }

  public Command getClimbCommand(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> climbState = state), CommandUtils.copyCommand(climbCommands))
        .withName("StateMachince/ClimbCommand/" + climbState);
  }

  public Command intakeNote() {
    return new ParallelCommandGroup(
        getAnglerCommand(AnglerStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getIndexerCommand(IndexerStates.STOW),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command podiumShot() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.SHOOT, AnglerStates.PODIUM));
  }

  public Command speakerShot() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.SHOOT, AnglerStates.SPEAKER));
  }

  public Command feedShot() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.SHOOT, AnglerStates.FEEDER));
  }

  public Command outtakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OUTTAKE),
        getIndexerCommand(IndexerStates.OUTDEX),
        getAnglerCommand(AnglerStates.INTAKE));
  }

  public Command stopTakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OFF),
        getIndexerCommand(IndexerStates.OFF),
        getShooterCommand(LauncherStates.OFF, AnglerStates.OFF));
  }

  public Command climbToAmp() {
    return getClimbCommand(ClimbStates.AMP);
  }

  public Command prepareNoteShot() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.SHOOT, AnglerStates.AIM));
  }

  public Command shootNote() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.INDEX), getLauncherCommand(LauncherStates.SHOOT));
  }

  public Command revUp() {
    return new ParallelCommandGroup(getLauncherCommand(LauncherStates.SHOOT));
  }

  public Command ejectSlow() {
    return getShooterCommand(LauncherStates.EJECT, AnglerStates.EJECT);
  }

  public Command runIndexer() {
    return getIndexerCommand(IndexerStates.INDEX);
  }

  public Command stopShooting() {
    return new ParallelCommandGroup(
        getIndexerCommand(IndexerStates.OFF), getLauncherCommand(LauncherStates.IDLE));
  }

  public Command moveAnglerUpManual() {
    return new ParallelCommandGroup(getAnglerCommand(AnglerStates.UP));
  }

  public Command moveAnglerDownManual() {
    return new ParallelCommandGroup(getAnglerCommand(AnglerStates.DOWN));
  }

  public Command shooterToIdle() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.IDLE, AnglerStates.IDLE));
  }

  public Command climbDown() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_UP), getAnglerCommand(AnglerStates.CLIMB));
  }

  public Command climbUp() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_DOWN), getAnglerCommand(AnglerStates.CLIMB));
  }

  public Command stopClimb() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.OFF), getAnglerCommand(AnglerStates.CLIMB));
  }

  public Command eject() {
    return new ParallelCommandGroup(getShooterCommand(LauncherStates.EJECT, AnglerStates.EJECT));
  }

  public Command revAmp() {
    return getShooterCommand(LauncherStates.REV_AMP, AnglerStates.AMP);
  }

  public Command scoreAmp() {
    return getLauncherCommand(LauncherStates.SCORE_AMP).andThen(getIndexerCommand(IndexerStates.AMP));
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
}
