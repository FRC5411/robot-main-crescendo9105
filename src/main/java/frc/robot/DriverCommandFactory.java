package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotStates.ClimbStates;
import frc.robot.RobotStates.IndexerStates;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.ShooterStates;

public class DriverCommandFactory {
    public StateMachine stateMachine;

    public DriverCommandFactory(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    public Command podiumShot() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.PODIUM));
    }

    public Command speakerShot() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.SPEAKER));
    }

    public Command feedShot() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.FEEDER));
    }

    public Command intakeNote() {
        return new ParallelCommandGroup(
            stateMachine.setShooterState(ShooterStates.INTAKE),
            stateMachine.setIntakeState(IntakeStates.INTAKE),
            stateMachine.setIndexerState(IndexerStates.STOW),
            stateMachine.setClimbState(ClimbStates.IDLE));
      }

    public Command outtakeNote() {
        return new ParallelCommandGroup(
            stateMachine.setIntakeState(IntakeStates.OUTTAKE),
            stateMachine.setIndexerState(IndexerStates.OUTDEX),
            stateMachine.setShooterState(ShooterStates.INTAKE));
    }

    public Command stopTakeNote() {
        return new ParallelCommandGroup(
            stateMachine.setIntakeState(IntakeStates.OFF),
            stateMachine.setIndexerState(IndexerStates.OFF),
            stateMachine.setShooterState(ShooterStates.OFF));
    }

    public Command climbToAmp() {
        return stateMachine.setClimbState(ClimbStates.AMP);
    }

    public Command prepareNoteShot() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.AIM));
    }

    public Command shootNote() {
        return new ParallelCommandGroup(
            stateMachine.setIndexerState(IndexerStates.INDEX), 
            stateMachine.setShooterState(ShooterStates.FIRE));
    }

    public Command revUp() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.FIRE));
    }

    public Command ejectSlow() {
        return stateMachine.setShooterState(ShooterStates.EJECT);
    }

    public Command runIndexer() {
        return stateMachine.setIndexerState(IndexerStates.INDEX);
    }

    public Command stopShooting() {
        return new ParallelCommandGroup(
            stateMachine.setIndexerState(IndexerStates.OFF), 
            stateMachine.setShooterState(ShooterStates.IDLE));
    }

    public Command moveAnglerUpManual() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.UP));
    }

    public Command moveAnglerDownManual() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.DOWN));
    }

    public Command shooterToIdle() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.IDLE));
    }

    public Command climbDown() {
        return new ParallelCommandGroup(
            stateMachine.setClimbState(ClimbStates.MOVE_BOTH_UP), stateMachine.setShooterState(ShooterStates.CLIMB));
    }

    public Command climbUp() {
        return new ParallelCommandGroup(
            stateMachine.setClimbState(ClimbStates.MOVE_BOTH_DOWN), stateMachine.setShooterState(ShooterStates.CLIMB));
    }

    public Command stopClimb() {
        return new ParallelCommandGroup(
            stateMachine.setClimbState(ClimbStates.OFF), stateMachine.setShooterState(ShooterStates.CLIMB));
    }

    public Command eject() {
        return new ParallelCommandGroup(stateMachine.setShooterState(ShooterStates.EJECT));
    }

    public Command revAmp() {
        return stateMachine.setShooterState(ShooterStates.AMP);
    }

    public Command scoreAmp() {
        return stateMachine.setIndexerState(IndexerStates.AMP);
    }
}
