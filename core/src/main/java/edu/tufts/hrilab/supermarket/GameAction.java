/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

public abstract class GameAction {

    protected boolean receivedResponse;
    protected String rawResponse;
    protected int maxResponseWait;
    protected static int RESP_SLEEP = 50;

    public GameAction(){
        this.receivedResponse = false;
        this.maxResponseWait = 5000;
    }

    public abstract  String getCommand();

    public String getRawResponse() {
        if (this.receivedResponse) {
            return this.rawResponse;
        } else {
            throw new RuntimeException("Response Not Received Yet");
        }
    }

//    TODO get rid of the action-specific response. A response should belong to a *joint* action.

    public int getMaxResponseWait(){
        return this.maxResponseWait;
    }

    public abstract boolean getSuccess();
    public abstract double getStepCost();
    public abstract boolean getGameOver();

    public void insertResponse(String response){
        if(this.receivedResponse){
            throw new RuntimeException("Response Already Received");
        }
        this.rawResponse = response;
        this.parseResponse();
        this.receivedResponse = true;
    }

    public void awaitResponse() {
        int max_time = this.getMaxResponseWait();
        int cycles = (int) Math.ceil(((float) max_time)/RESP_SLEEP);

        for (int i = 0; i < cycles; i++){
            if(this.receivedResponse) {
                break;
            }
            try {
                Thread.sleep(RESP_SLEEP);
            }catch (InterruptedException e){
                return;
            }
        }
    }

    protected abstract void parseResponse();

}