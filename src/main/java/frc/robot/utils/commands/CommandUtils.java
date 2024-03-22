package frc.robot.utils.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static SendableChooser<Command> buildAutoChooser() {
        return buildAutoChooser("");
    }

      // Build auto chooser method that catches null autos
    public static SendableChooser<Command> buildAutoChooser(String defaultAutoName) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> autoNames = AutoBuilder.getAllAutoNames();

        PathPlannerAuto defaultOption = null;
        List<PathPlannerAuto> options = new ArrayList<>();

        for (String autoName : autoNames) {
          try{
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
                defaultOption = auto;
            } else {
                options.add(auto);
            }
          } catch (Exception e) {
            System.out.println("Auto " + autoName + " failed to load");
            chooser.addOption("FAILEDTOLOAD: "+autoName, defaultOption);
          }
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", Commands.none());
        } else {
            chooser.setDefaultOption(defaultOption.getName(), defaultOption);
        }

        options.forEach(auto -> chooser.addOption(auto.getName(), auto));

        return chooser;
    }
}
