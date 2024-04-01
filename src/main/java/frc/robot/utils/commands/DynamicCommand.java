package frc.robot.utils.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DynamicCommand extends Command {
    public Supplier<Command> commandSupplier;
    public Command currentCommand;

    // Do not use commands that have different requirements than the passed in subsystem requriements
    public DynamicCommand(Supplier<Command> commandSupplier, Subsystem... requirements) {
        this.commandSupplier = commandSupplier;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        currentCommand = commandSupplier.get();
        if(currentCommand == null) {
            currentCommand = new PrintCommand("DynamicCommand: Command supplier returned null");
        }
        currentCommand.initialize();
    }

    @Override
    public void execute() {
        currentCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        if(currentCommand == null) {
            return false;
        }
        return currentCommand.runsWhenDisabled();
    }

    public Command copy() {
        return new DynamicCommand(commandSupplier, getRequirements().toArray(new Subsystem[0]));
    }
}