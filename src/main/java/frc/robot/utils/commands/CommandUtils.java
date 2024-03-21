package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandUtils {
  public static Command copyCommand(Command command) {
    return new FunctionalCommand(
        command::initialize,
        command::execute,
        command::end,
        command::isFinished,
        command.getRequirements().toArray(Subsystem[]::new));
  }
}
