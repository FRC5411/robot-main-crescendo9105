package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbVoltSetpoints;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.AnglerSetpoints;
import frc.robot.subsystems.shooter.Shooter.LauncherSetpoints;
import frc.robot.utils.commands.CommandUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.RobotStates.YoshiStates;
import frc.robot.subsystems.yoshivator.Yoshivator.YoshivatorSetpoints;

public class StateMachine {
  private Shooter robotShooter;
  private Intake robotIntake;
  private Climb robotClimb;
  private Yoshivator robotYoshi;

  private ShooterStates shooterState;
  private IntakeStates intakeState;
  private ClimbStates climbState;
  private YoshiStates yoshiState;

  private SelectCommand<ShooterStates> shooterCommands;
  private SelectCommand<IntakeStates> intakeCommands;
  private SelectCommand<ClimbStates> climbCommands;
  private SelectCommand<YoshiStates> yoshiCommands;

  public StateMachine(
      Shooter robotShooter, Intake robotIntake, Climb robotClimb, Yoshivator robotYoshi) {
    this.robotShooter = robotShooter;
    this.robotIntake = robotIntake;
    this.robotClimb = robotClimb;
    this.robotYoshi = robotYoshi;

    shooterState = ShooterStates.IDLE;
    intakeState = IntakeStates.OFF;
    climbState = ClimbStates.OFF;
    yoshiState = YoshiStates.IDLE;

    shooterCommands =
        new SelectCommand<ShooterStates>(this.robotShooter.mapToCommand(), () -> shooterState);

    intakeCommands =
        new SelectCommand<IntakeStates>(this.robotIntake.mapToCommand(), () -> intakeState);

    climbCommands =
        new SelectCommand<ClimbStates>(this.robotClimb.mapToCommand(), () -> climbState);

    yoshiCommands =
        new SelectCommand<YoshiStates>(this.robotYoshi.mapToCommand(yoshiState), () -> yoshiState);

    robotShooter.setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.IDLE);
    robotIntake.setCurrentSetpoint(IntakeSetpoint.OFF);
    robotClimb.setManualVolts(ClimbVoltSetpoints.OFF);
    robotYoshi.setYoshiSetpoint(YoshivatorSetpoints.IDLE);
  }

  public Command getShooterCommand(ShooterStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> shooterState = state),
            CommandUtils.copyCommand(shooterCommands))
        .withName("StateMachince/ShooterCommand/" + shooterState);
  }

  public Command getIntakeCommand(IntakeStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeState = state), CommandUtils.copyCommand(intakeCommands))
        .withName("StateMachince/IntakeCommand/" + intakeState);
  }

  public Command getClimbCommand(ClimbStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> climbState = state), CommandUtils.copyCommand(climbCommands))
        .withName("StateMachince/ClimbCommand/" + climbState);
  }

  public Command getYoshiCommand(YoshiStates state) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> yoshiState = state), CommandUtils.copyCommand(yoshiCommands))
        .withName("StateMachince/YoshiCommand/" + yoshiState);
  }

  public Command intakeNote() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getClimbCommand(ClimbStates.IDLE));
  }

  public Command yoshiIntakeNote() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.INTAKE),
        getIntakeCommand(IntakeStates.INTAKE),
        getClimbCommand(ClimbStates.IDLE),
        getYoshiCommand(YoshiStates.INTAKE));
  }

  public Command outtakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OUTTAKE),
        getShooterCommand(ShooterStates.INTAKE));
  }

  public Command stopTakeNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.OFF),
        getShooterCommand(ShooterStates.OFF),
        getYoshiCommand(YoshiStates.IDLE));
  }

  public Command podiumShot() {
    return getShooterCommand(ShooterStates.PODIUM);
  }

  public Command speakerShot() {
    return getShooterCommand(ShooterStates.SPEAKER);
  }

  public Command feedShot() {
    return getShooterCommand(ShooterStates.FEEDER);
  }

  public Command climbToAmp() {
    return getClimbCommand(ClimbStates.AMP);
  }

  public Command prepareNoteShot() {
    return getShooterCommand(ShooterStates.AIM);
  }

  public Command shootNote() {
    return new ParallelCommandGroup(
        getIntakeCommand(IntakeStates.INTAKE));
  }

  public Command revUp() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.FIRE));
  }

  public Command ejectSlow() {
    return getShooterCommand(ShooterStates.EJECT);
  }

  public Command stopShooting() {
    return new ParallelCommandGroup(
        getShooterCommand(ShooterStates.IDLE), 
        getIntakeCommand(IntakeStates.OFF));
  }

  public Command moveAnglerUpManual() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.UP));
  }

  public Command moveAnglerDownManual() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.DOWN));
  }

  public Command shooterToIdle() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.IDLE));
  }

  public Command climbDown() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_UP), 
        getShooterCommand(ShooterStates.CLIMB));
  }

  public Command climbUp() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.MOVE_BOTH_DOWN), 
        getShooterCommand(ShooterStates.CLIMB));
  }

  public Command stopClimb() {
    return new ParallelCommandGroup(
        getClimbCommand(ClimbStates.OFF), 
        getShooterCommand(ShooterStates.CLIMB));
  }

  public Command eject() {
    return new ParallelCommandGroup(getShooterCommand(ShooterStates.EJECT));
  }

  public Command revAmp() {
    return getShooterCommand(ShooterStates.REV_AMP);
  }

  public Command scoreAmp() {
    return getShooterCommand(ShooterStates.SHOOT_AMP).andThen(
      new ParallelCommandGroup(getIntakeCommand(IntakeStates.AMP)));
  }

  @AutoLogOutput(key = "StateMachine/ShooterState")
  public ShooterStates getShooterState() {
    return shooterState;
  }

  @AutoLogOutput(key = "StateMachine/IntakeState")
  public IntakeStates getIntakeState() {
    return intakeState;
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
