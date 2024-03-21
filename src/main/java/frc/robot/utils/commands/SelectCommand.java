package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.function.Supplier;

public class SelectCommand<K> extends Command {
  private HashMap<K, Command> commandMap;
  private Supplier<K> stateSupplier;
  private Command selectedCommand;
  private Command defaultCommand;

  public SelectCommand(
      HashMap<K, Command> commandMap, Supplier<K> stateSupplier, Command defaultCommand) {
    this.commandMap = commandMap;
    this.stateSupplier = stateSupplier;
    this.defaultCommand = defaultCommand;

    for (Command command : commandMap.values()) {
      addRequirements(command.getRequirements().toArray(Subsystem[]::new));
    }
  }

  @Override
  public void initialize() {
    selectedCommand = commandMap.getOrDefault(stateSupplier.get(), defaultCommand);
    if (selectedCommand == null) {
      selectedCommand = defaultCommand;
    }
    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return selectedCommand.runsWhenDisabled();
  }

  public void setDefaultCommand(Command defaultCommand) {
    this.defaultCommand = defaultCommand;
  }

  // This function makes a copy of this command and returns it
  // This is done to avoid command composition erros
  public Command copy() {
    return CommandUtils.copyCommand(this);
  }
}
