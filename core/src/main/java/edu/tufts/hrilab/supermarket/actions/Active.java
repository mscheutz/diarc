/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket.actions;

import com.google.gson.Gson;
import edu.tufts.hrilab.supermarket.GameAction;
import edu.tufts.hrilab.supermarket.SupermarketObservation;

public abstract class Active extends GameAction {
    protected ParsedResponse parsedResponse;

    protected class ParsedResponse {
        CommandResult command_result;
        int step;
        boolean gameOver;
        SupermarketObservation observation;

        protected class CommandResult {
            String command;
            String result;
            String message;
            double stepCost;
        }
    }

    public Active(){ }

    public SupermarketObservation getObservation() {
        return this.parsedResponse.observation;
    }

    public boolean getSuccess() {
        return this.parsedResponse.command_result.result.equals("SUCCESS");
    }

    public double getStepCost() {
        return this.parsedResponse.command_result.stepCost;
    }

    public String getMessage() {return this.parsedResponse.command_result.message; }

    public boolean getGameOver() { return this.parsedResponse.gameOver; }

    protected void parseResponse() {
        
        this.parsedResponse = new Gson().fromJson(this.rawResponse, ParsedResponse.class);

//        if(!getCommand().equals(commandSent)) {
//            throw new RuntimeException("Wrong Response Received");
//        }
    }
}
