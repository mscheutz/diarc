package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import org.json.JSONArray;
import org.json.JSONObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.util.*;

@Component
public class GoalEndpoint extends TextWebSocketHandler {
    private static final Logger log =
            LoggerFactory.getLogger(GoalEndpoint.class);
    private final GoalManagerImpl goalManager;
    private WebSocketSession session;

    private Timer updateTimer;
    public static final int UPDATE_PERIOD = 1000; // in milliseconds

    /**
     * Constructs the <code>GoalEndpoint</code> given a goal manager component.
     * @param goalManager the goal manager whose goals will be displayed
     */
    public GoalEndpoint(GoalManagerImpl goalManager) {
        this.goalManager = goalManager;
    }

    /**
     * Converts a list of goals into a JSON object where the keys are the
     * goals' agents and the values are JSON arrays of those agents' goals.
     * @param goals a list of goals to convert
     * @return a JSONObject of goals, grouped by agent
     */
    private JSONObject goals2JSONObject(List<Goal> goals) {
        if(goals.isEmpty()) return new JSONObject(); // no goals
        JSONObject result = new JSONObject();

        // Group goals by actor
        goals.sort(Comparator.comparing((Goal o) ->
                o.getActor().toString()));

        Symbol groupActor = goals.get(0).getActor();
        JSONArray group = new JSONArray();
        group.put(goals.get(0).toString());
        for(int i = 1; i < goals.size(); i++) {
            Symbol currentActor = goals.get(i).getActor();
            // If actor matches group, add it
            if(groupActor.equals(currentActor)) {
                group.put(goals.get(i).toString());
            }
            // If actor is new, then update JSON and start new group
            else {
                result.put(groupActor.toString(), group);
                group = new JSONArray();
                group.put(goals.get(i).toString());
                groupActor = goals.get(i).getActor();
            }
        }
        result.put(groupActor.toString(), group);
        return result;
    }

    /**
     * Updates the user about the current goals of the robots.
     */
    public void updateGoals() {
        try {
            if (session != null) {
                JSONObject active = goals2JSONObject(this.goalManager
                        .getExecutionManager().getActiveGoals());
                JSONObject past = goals2JSONObject(this.goalManager
                        .getExecutionManager().getPastGoals());
                JSONObject all = new JSONObject()
                        .put("active", active)
                        .put("past", past);
                session.sendMessage(new TextMessage(all.toString()));
            } else {
                log.error("Could not send message: no active session");
            }
        } catch (Exception e) {
            log.error(e.getMessage());
        }
    }

    //======================
    // TextWebSocketHandler
    //======================

    /**
     * Starts a periodic update after the connection is established to the GUI
     * client.
     * @param session the web socket session this endpoint communicates over
     */
    @Override
    public void afterConnectionEstablished(@Nonnull WebSocketSession session) {
        this.session = session;
        log.info("Connection established");

        updateTimer = new Timer();
        updateTimer.scheduleAtFixedRate(new TimerTask() {
            public void run() {
                updateGoals();
            }
        }, 0, UPDATE_PERIOD);
    }

    /**
     * Performs cleanup when the connection is closed.
     * @param session the session that is ending
     * @param status close status of the connection
     * @throws Exception ignored
     */
    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status)
            throws Exception {
        super.afterConnectionClosed(session, status);
        this.session = null;
        updateTimer.cancel();
    }
}
