/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.supermarket.actions.Active;
import edu.tufts.hrilab.supermarket.actions.SetObservation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.socket.SocketConnection;
import java.io.IOException;

class GamePlay
{
    int ACTION_WAIT = 20;
    SocketConnection sock;
    SupermarketObservation observation = null;
    private static final Logger log = LoggerFactory.getLogger(GamePlay.class);


    public GamePlay(int socketPort){
        try {
            this.sock = new SocketConnection(socketPort);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    public void setObservation(SupermarketObservation obs) {
        String json = new GsonBuilder().serializeNulls().create().toJson(obs);
        this.sock.sendCommand("SET " + json);
        String response = this.sock.waitedResponse(1000);

        GameAction action = new SetObservation();
        action.insertResponse(response);

        this.observation = ((Active)action).getObservation();
    }

    public void perform(GameAction action, int playerIndex) {
        this.sock.sendCommand(getCommand(action, playerIndex));
        String response = this.sock.waitedResponse(1000);

        action.insertResponse(response);

        this.observation = ((Active)action).getObservation();
    }

    private String getCommand(GameAction action, int playerIndex) {
        return playerIndex + " " + action.getCommand();
    }
}