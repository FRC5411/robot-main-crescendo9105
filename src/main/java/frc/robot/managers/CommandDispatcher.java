package frc.robot.managers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.managers.RobotDesiredStates.AnglerStates;
import frc.robot.managers.RobotDesiredStates.ClimbStates;
import frc.robot.managers.RobotDesiredStates.IndexerStates;
import frc.robot.managers.RobotDesiredStates.IntakeStates;
import frc.robot.managers.RobotDesiredStates.LauncherStates;
import frc.robot.managers.RobotDesiredStates.YoshiStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.managers.RobotSetpoints.ClimPosSetpoints;
import frc.robot.managers.RobotSetpoints.ClimbVoltSetpoints;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.managers.RobotSetpoints.IndexerSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.managers.RobotSetpoints.IntakeSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.managers.RobotSetpoints.AnglerSetpoints;
import frc.robot.managers.RobotSetpoints.LauncherSetpoints;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.subsystems.shooter.launcher.Launcher;
import frc.robot.utils.commands.DynamicCommand;

import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.managers.RobotSetpoints.YoshivatorSetpoints;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandDispatcher {
  private Launcher robotLauncher;
  private Angler robotAngler;
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

  public CommandDispatcher(
      Shooter robotShooter, Intake robotIntake, Indexer robotIndexer, Climb robotClimb, Yoshivator robotYoshi) {
    this.robotLauncher = robotShooter.getLauncher();
    this.robotAngler = robotShooter.getAngler();
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
        () -> {
          switch(getLauncherState()) {
            case OFF:
              return Commands.runOnce(() -> robotLauncher.stopMotors(), robotLauncher);
            case SPEAKER_SHOT:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.SPEAKER_SHOT);
            case IDLE:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.IDLE);
            case FEEDER:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.FEEDER);
            case REV_AMP:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.AMP);
            case SHOOT_AMP:
              return Commands.runOnce(() -> {
                robotLauncher.setLauncherVelocityMPS(null);
                robotLauncher.setTopLauncherVolts(0);
              }, robotLauncher);
            case EJECT:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.EJECT);
            case FULL_SPEED:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.FULL_SPEED);
            default:
              return robotLauncher.setVelocityMPS(LauncherSetpoints.IDLE);
          }
        }, this.robotLauncher);

    anglerCommands = new DynamicCommand(() -> {
      switch(anglerState) {
        case OFF:
          return Commands.runOnce(() -> robotAngler.stopMotors(), robotAngler);
        case AIM:
          return robotAngler.setAnglerCommand(AnglerSetpoints.AIM);
        case INTAKE:
          return robotAngler.setAnglerCommand(AnglerSetpoints.INTAKE);
        case CLIMB:
          return robotAngler.setAnglerCommand(AnglerSetpoints.CLIMB);
        case EJECT:
          return robotAngler.setAnglerCommand(AnglerSetpoints.FEEDER);
        case UP:
          return robotAngler.setAnglerManualVolts(5.0);
        case DOWN:
          return robotAngler.setAnglerManualVolts(-5.0);
        case IDLE:
          return robotAngler.setAnglerCommand(AnglerSetpoints.IDLE);
        case PODIUM:
          return robotAngler.setAnglerCommand(AnglerSetpoints.PODIUM);
        case SPEAKER:
          return robotAngler.setAnglerCommand(AnglerSetpoints.SPEAKER);
        case FEEDER:
          return robotAngler.setAnglerCommand(AnglerSetpoints.FEEDER);
        case AMP:
          return robotAngler.setAnglerCommand(AnglerSetpoints.AMP);
        default:
          return robotAngler.setAnglerCommand(AnglerSetpoints.IDLE);
      }
    }, robotAngler);
    
    intakeCommands =
      new DynamicCommand(
        () -> {
          switch(getIntakeState()) {
            case INTAKE:
              return this.robotIntake.runIntake(IntakeSetpoint.IN);
            case OUTTAKE:
              return this.robotIntake.runIntake(IntakeSetpoint.OUT);
            case OFF:
              return this.robotIntake.stopIntake();
            case AUTO_INTAKE:
              return this.robotIntake.runIntake(IntakeSetpoint.AUTO_IN);
            default:
              return this.robotIntake.stopIntake();
          }}, this.robotIntake);

    indexerCommands =
      new DynamicCommand(
        () -> {
          switch(getIndexerState()) {
            case INDEX:
              return this.robotIndexer.runIndexer(IndexerSetpoint.IN);
            case OUTDEX:
              return this.robotIndexer.runIndexer(IndexerSetpoint.OUT);
            case STOW:
              return this.robotIndexer.stowPiece();
            case OFF:
              return this.robotIndexer.runIndexer(IndexerSetpoint.OFF);
            case AMP:
              return this.robotIndexer.runIndexer(IndexerSetpoint.AMP);
            default:
              return this.robotIndexer.runIndexer(IndexerSetpoint.OFF);
          }}, this.robotIndexer);
    
    climbCommands =
      new DynamicCommand(
        () -> {
          switch (getClimbState()) {
            case IDLE:
              return this.robotClimb.setPositionSetpoint(ClimPosSetpoints.IDLE.getRotation());
            case OFF:
              return this.robotClimb.runVolts(ClimbVoltSetpoints.OFF);
            case MOVE_BOTH_UP:
              return this.robotClimb.runVolts(ClimbVoltSetpoints.BOTH_UP);
            case MOVE_BOTH_DOWN:
              return this.robotClimb.runVolts(ClimbVoltSetpoints.BOTH_DOWN);
            default:
              return this.robotClimb.runVolts(ClimbVoltSetpoints.OFF);
          }}, this.robotClimb);

    yoshiCommands =
      new DynamicCommand(
        () -> {
          switch(getYoshiState()) {
            case INTAKE:
              return this.robotYoshi.runYoshi(YoshivatorSetpoints.GROUND_INTAKE);
            case IDLE:
              return this.robotYoshi.runYoshi(YoshivatorSetpoints.IDLE);
            case OFF:
              return this.robotYoshi.offYoshi();
            default:
                return this.robotYoshi.offYoshi();
          }}, this.robotYoshi);

    robotLauncher.setLauncherVelocityMPS(LauncherSetpoints.OFF);
    robotAngler.setAnglerPosition(AnglerSetpoints.IDLE);
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
        getYoshiCommand(YoshiStates.INTAKE));
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