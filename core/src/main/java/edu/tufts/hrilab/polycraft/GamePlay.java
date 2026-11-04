/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft;

import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.polycraft.actions.GameAction;
import edu.tufts.hrilab.polycraft.actions.sense.Sense;
import edu.tufts.hrilab.polycraft.actions.sense.SenseAll;
import edu.tufts.hrilab.socket.SocketConnection;
import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public final class GamePlay {
    static private Logger log = LoggerFactory.getLogger(GamePlay.class);
    private final int ACTION_WAIT = 300;
    private SocketConnection sock;
    /**
     * Flag indicating if game state has been updated.
     */
    private volatile boolean gameStateUpdated = false;
    /**
     * Most recent game state from SenseAll.
     */
    private Prolog gameState = null;
    /**
     * Most recent SenseAll used to populate gameState.
     */
    private Sense lastSenseAction = null;
    /**
     * Cost of the most recent action taken.
     */
    private Map<String,GameAction> lastActionsTaken = new HashMap<>();

    public GamePlay(int port) {
        try {
            this.sock = new SocketConnection(port);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Reset cached game info/state between games.
     */
    public void reset() {
        gameState = new Prolog();
        lastSenseAction = null;
        lastActionsTaken = new HashMap<>();
        gameStateUpdated = false;
    }

    /**
     * Check if game state has changes since game state was last updated.
     * @return
     */
    public synchronized boolean hasGameStateChanged() {
        return !gameStateUpdated;
    }

    /**
     * Get the most recent SenseAll used to populate the gameState prover.
     * @return
     */
    public synchronized Sense getLastSenseAction() {
        if (!gameStateUpdated) {
            log.warn("Returning stale Sense action. Need to call updateGameState.");
        }
        return lastSenseAction;
    }

    /**
     * Update the game state by calling SenseAll.
     */
    public synchronized void updateGameState() {
        if (gameStateUpdated) {
            log.warn("Game state already updated.");
        } else {
            Sense newSenseAction = new SenseAll();
            perform(newSenseAction);

            lastSenseAction = newSenseAction;

            // TODO: this is really inefficient -- come up with better approach for updating observation/game state
            gameState = new Prolog();
            newSenseAction.getAssertions().forEach(predicate -> gameState.assertBelief(predicate));

            gameStateUpdated = true;
        }
    }

    /**
     * Get the current game state (from SenseAll) and
     * return a Prolog instance populated with senseall results.
     * @return
     */
    public synchronized Prover getGameState() {
        if (!gameStateUpdated) {
            log.warn("Returning stale game state. Need to call updateGameState.");
        }
        return gameState;
    }

    public synchronized GameAction getLastActionTaken(String command) {
        GameAction action = lastActionsTaken.get(command);
        if (action == null) {
            log.warn("Action not found with command "+command);
            return null;
        }

        return action;
    }


    public synchronized void perform(GameAction action) {
        String command = action.getCommand();
        log.debug("Calling polycraft with action: " + command);

        // update last action
        int i = command.length();
        if (command.contains(" ")){
            i = command.indexOf(" ");
        }
        lastActionsTaken.put(command.substring(0,i),action);

        this.sock.sendCommand(command);
        action.insertResponse(this.sock.waitedResponse(action.getMaxResponseWait()));

        Util.Sleep(ACTION_WAIT);

        if (action.canChangeGameState()) {
            gameStateUpdated = false;
        }
    }

}
