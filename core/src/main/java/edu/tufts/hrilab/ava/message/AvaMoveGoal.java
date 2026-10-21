package edu.tufts.hrilab.ava.message;

import edu.tufts.hrilab.fol.Symbol;

import java.util.Arrays;
import java.util.List;

public class AvaMoveGoal {
    private CommandType commandType;
    private List<Object> parameters;

    public AvaMoveGoal() {
        setIdleGoal();
    }

    public AvaMoveGoal(CommandType commandType, Object... parameters) {
        this.commandType = commandType;
        this.parameters = Arrays.asList(parameters);
    }

    public CommandType getCommandType() {
        return commandType;
    }

    public List<Object> getParameters() {
        return parameters;
    }

    public String toString() {
        String ret = commandType.name();
        for (Object parameter : parameters) {
            ret += " " + parameter.toString();
        }
        return ret;
    }

    public void setGoToLocationGoal(Symbol location, boolean wait) {
        if (commandType == CommandType.IDLE) {
            commandType = CommandType.GO_TO_LOCATION;
            parameters = List.of(location, wait);
        }
    }

    public void setGoHomeGoal() {
        commandType = CommandType.GO_HOME;
        parameters = List.of();
    }

    public void setIdleGoal(CommandType callFrom) {
        if (callFrom == commandType) {
            setIdleGoal();
        }
    }

    private void setIdleGoal() {
        commandType = CommandType.IDLE;
        parameters = List.of();
    }
    public enum CommandType {
        GO_TO_LOCATION,
        GO_HOME,
        IDLE
    }

    public boolean isIdle() {
        return commandType == CommandType.IDLE;
    }

    public boolean isGoHome() {
        return commandType == CommandType.GO_HOME;
    }

    public boolean isGoToLocation() {
        return commandType == CommandType.GO_TO_LOCATION;
    }
}




