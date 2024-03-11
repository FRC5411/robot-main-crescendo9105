package frc.robot;

import frc.robot.commands.IntakeCommands;
import frc.robot.commands.IndexerCommands;

import frc.robot.subsystems.yoshivator.Yoshivator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.Command;

public class GamePieceCollector {
    private Intake robotIntake;
    private Indexer robotIndexer;
    private Yoshivator robotYoshi;

    public GamePieceCollector(Intake intake, Indexer indexer, Yoshivator yoshi) {
        robotIntake = intake;
        robotIndexer = indexer;
        robotYoshi = yoshi;
    }

    public Command getIntakeCommand(IntakeSetpoints setpoint) {
        switch(setpoint) {
            case IN:
                return IntakeCommands.runIntake(robotIntake, IntakeCommands.IntakeDirection.IN);
            case OUT:
                return IntakeCommands.runIntake(robotIntake, IntakeCommands.IntakeDirection.OUT);
            case STOP:
            default:
                return IntakeCommands.stopIntake(robotIntake);
        }
    }

    public Command getIndexerCommand(IndexerSetpoints setpoint) {
        switch(setpoint) {
            case IN:
                return IndexerCommands.runIndexer(robotIndexer, IndexerCommands.IndexerDirection.IN);
            case OUT:
                return IndexerCommands.runIndexer(robotIndexer, IndexerCommands.IndexerDirection.OUT);
            case STOP:
            default:
                return IndexerCommands.stopIndexer(robotIndexer);
        }
    }

    public Command getYoshiCommand(YoshiSetpoints setpoint) {
        return null;
    }

    public enum IntakeSetpoints {
        IN, OUT, STOP
    }

    public enum IndexerSetpoints {
        IN, OUT, STOW, STOP
    }

    public enum YoshiSetpoints {
        IDLE, GROUND, AMP
    }

    public enum PivotSetpoints {
        IDLE, GROUND, AMP
    }
}