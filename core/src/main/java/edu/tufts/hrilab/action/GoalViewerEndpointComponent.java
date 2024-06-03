package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import org.json.JSONArray;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.util.*;

@Component
public class GoalViewerEndpointComponent extends DiarcComponent {
    private final GoalViewerHandler goalViewerHandler;

    /**
     * Constructor.
     */
    public GoalViewerEndpointComponent() {
        this.goalViewerHandler = new GoalViewerHandler();
    }

    /**
     * Getter for the goal viewer handler instance.
     * @return the goal viewer handler
     */
    @TRADEService
    public GoalViewerHandler getGoalViewerHandler() {
        return goalViewerHandler;
    }

    /**
     * Updates the user about the current goals of the robots.
     */
    @TRADEService
    public void updateGoals() {
        this.goalViewerHandler.updateGoals();
    }

    /**
     * GoalViewerHandler inner class. This implements the server.
     */
    public class GoalViewerHandler extends TextWebSocketHandler {
        private Timer updateTimer;
        public static final int UPDATE_PERIOD = 1000; // in milliseconds

        TRADEServiceInfo getActiveGoals;
        TRADEServiceInfo getPastGoals;

        private WebSocketSession session;

        /**
         * Constructor.
         */
        public GoalViewerHandler() {
            getActiveGoals = null;
            getPastGoals = null;
            Collection<TRADEServiceInfo> services = TRADE.getAvailableServices();
            for(TRADEServiceInfo service : services) {
                if(service.serviceString.equals("getActiveGoals()")) {
                    getActiveGoals = service;
                } else if(service.serviceString.equals("getPastGoals()")) {
                    getPastGoals = service;
                }

                if(getActiveGoals != null && getPastGoals != null)
                    break;
            }

            if(getActiveGoals == null || getPastGoals == null)
                log.error("Failed to find getter for active or passive goals");
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
        @SuppressWarnings("unchecked assignment")
        public void updateGoals() {
            try {
                if (session != null) {
                    JSONObject active = goals2JSONObject(
                            getActiveGoals.call(List.class));
                    JSONObject past = goals2JSONObject(
                            getPastGoals.call(List.class));
                    JSONObject all = new JSONObject()
                            .put("active", active)
                            .put("past", past);
                    session.sendMessage(new TextMessage(all.toString()));
                } else {
                    log.error("Could not send message: no active session");
                    this.session = null;
                    updateTimer.cancel();
                }
            } catch (Exception e) {
                log.error(e.getMessage());
            }
        }

        //======================
        // TextWebSocketHandler
        //======================

        /**
         * Handle a text message from the user.
         *
         * @param session the web socket session the message comes over on
         * @param message the message from the user
         * @throws Exception ignored
         */
        @Override
        protected void handleTextMessage(@Nonnull WebSocketSession session,
                        @Nonnull TextMessage message) throws Exception {
            super.handleTextMessage(session, message);
            this.session = session;
            // Don't need to send anything
        }

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
        public void afterConnectionClosed(@Nonnull WebSocketSession session,
                        @Nonnull CloseStatus status) throws Exception {
            super.afterConnectionClosed(session, status);
            this.session = null;
            updateTimer.cancel();
        }
    }
}
