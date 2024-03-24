package frc.robot.managers.statemachine;

import frc.robot.CommandDispatcher;

public class StateManager {
    private CommandDispatcher dispatcher;
    
    public StateManager(CommandDispatcher dispatcher) {
        this.dispatcher = dispatcher;
    }
}
