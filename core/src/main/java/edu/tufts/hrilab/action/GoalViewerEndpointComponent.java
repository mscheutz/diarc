package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
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

/**
 * Wraps a text web socket handler in a DIARC component. Handles a server which
 * sits between TRADE's <code>ExecutionManager</code> and the frontend goal
 * viewer GUI.
 * @author Lucien
 * @version 1.0
 */
@Component
public class GoalViewerEndpointComponent extends DiarcComponent {
    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * This component's instance of the inner class.
     */
    private final GoalViewerHandler goalViewerHandler;

    /**
     * TRADE service to get the current active goals.
     */
    TRADEServiceInfo getActiveGoals;
    /**
     * TRADE service to get past goals.
     */
    TRADEServiceInfo getPastGoals;
    /**
     * TRADE service to submit a goal.
     */
    TRADEServiceInfo submitGoal;
    /**
     * TRADE service to cancel a goal.
     */
    TRADEServiceInfo cancelGoal;

    //==========================================================================
    // Constructor
    //==========================================================================
    /**
     * Constructor. Instiantiates the inner class.
     */
    public GoalViewerEndpointComponent() {
        this.goalViewerHandler = new GoalViewerHandler();
        initializeServices();
    }

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Find all TRADE services required.
     */
    private void initializeServices() {
        try {
            getActiveGoals = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("getActiveGoals")
                            .argTypes()
                            .returnType(List.class)
            );
            getPastGoals = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("getPastGoals")
                            .returnType(List.class)
            );

            cancelGoal = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("cancelGoal")
                            .argTypes(Long.class)
            );
            submitGoal = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("submitGoal")
                            .argTypes(Predicate.class)
                            .returnType(long.class)
            );
        } catch(TRADEException e) {
            log.error("Failed to get TRADE services for goal viewer");
        }
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

    //==========================================================================
    // Inner class | GoalViewerHandler
    //==========================================================================
    /**
     * Inner class which handles all server functions.
     */
    public class GoalViewerHandler extends TextWebSocketHandler {
        //======================================================================
        // Constants
        //======================================================================
        /**
         * Timer update period, in milliseconds.
         */
        public static final int UPDATE_PERIOD = 1000;

        //======================================================================
        // Fields
        //======================================================================
        /**
         * Timer that periodically updates the client with current goal info.
         */
        private Timer updateTimer;

        /**
         * Current session.
         */
        private WebSocketSession session;

        //======================================================================
        // Methods
        //======================================================================
        /**
         * Removes goals from the given list if they don't match the given
         * status.
         * @param goals list of goals to filter
         * @param status desired status
         */
        private void filterListByGoalStatus(List<Goal> goals, GoalStatus status) {
            List<Goal> toRemove = new ArrayList<>();
            for(Goal g : goals)
                if(g.getStatus() != status)
                    toRemove.add(g);
            goals.removeAll(toRemove);
        }

        /**
         * Converts a list of goals into a JSON array of JSON objects. Each
         * JSON object has a "name" field storing the agent's name and a
         * "children" field storing the agent's goals as an array. Each goal
         * is a JSON object with its own "name" field.
         * @param goals a list of goals to convert
         * @param goalType the group the goals belong to; can be
         *                 <code>"past"</code>, <code>"active"</code>, or
         *                 <code>"suspended"</code>. If <code>"active"</code> or
         *                 <code>"suspended"</code>, will filter active goals
         *                 for that specific <code>GoalStatus</code>.
         * @throws IllegalArgumentException if <code>goalType</code> is not
         *                                  a legal value
         * @return a JSON array
         */
        private JSONArray goals2JSONArray(List<Goal> goals, String goalType)
            throws IllegalArgumentException {
            if(goalType.equals("active"))
                filterListByGoalStatus(goals, GoalStatus.ACTIVE);
            else if(goalType.equals("suspended"))
                filterListByGoalStatus(goals, GoalStatus.SUSPENDED);
            else if(!goalType.equals("past"))
                throw new IllegalArgumentException(
                        "value of goalType is not accepted");

            // No goals
            if(goals.isEmpty()) return new JSONArray();
            JSONArray result = new JSONArray();

            // Group goals by actor
            goals.sort(Comparator.comparing((Goal o) ->
                    o.getActor().toString()));

            Symbol groupActor = goals.get(0).getActor();
            JSONArray group = new JSONArray();
            group.put(new JSONObject()
                    .put("name", goals.get(0).toString())
                    .put("id", "goal" + goals.get(0).getId()));
            for(int i = 1; i < goals.size(); i++) {
                Symbol currentActor = goals.get(i).getActor();
                // If actor matches group, add it
                if(groupActor.equals(currentActor)) {
                    group.put(new JSONObject()
                            .put("name", goals.get(i).toString())
                            .put("id", "goal" + goals.get(i).getId()));
                }
                // If actor is new, then update JSON and start new group
                else {
                    result.put(new JSONObject()
                            .put("name", groupActor.toString())
                            .put("children", group)
                            .put("id", goalType + groupActor));

                    group = new JSONArray();
                    group.put(new JSONObject()
                            .put("name", goals.get(i).toString())
                            .put("id", "goal" + goals.get(i).getId()));
                    groupActor = goals.get(i).getActor();
                }
            }
            result.put(new JSONObject()
                    .put("name", groupActor.toString())
                    .put("children", group)
                    .put("id", goalType + groupActor));
            return result;
        }

        /**
         * Updates the user about the current goals of the robots.
         */
        @SuppressWarnings("unchecked assignment")
        public void updateGoals() {
            try {
                if (session != null) {
                    JSONObject active = new JSONObject()
                            .put("name", "active")
                            .put("children", goals2JSONArray(
                                    getActiveGoals.call(List.class),
                                    "active")
                            )
                            .put("id", "active");
                    JSONObject suspended = new JSONObject()
                            .put("name", "suspended")
                            .put("children", goals2JSONArray(
                                    getActiveGoals.call(List.class),
                                    "suspended")
                            )
                            .put("id", "suspended");
                    JSONObject past = new JSONObject()
                            .put("name", "past")
                            .put("children", goals2JSONArray(
                                    getPastGoals.call(List.class),
                                    "past")
                            )
                            .put("id", "past");
                    JSONArray children = new JSONArray()
                            .put(active)
                            .put(suspended)
                            .put(past);
                    JSONObject all = new JSONObject()
                            .put("name", "root")
                            .put("id", "root")
                            .put("children", children);
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

        //======================================================================
        // Implementing methods | TextWebSocketHandler
        //======================================================================
        /**
         * Handle a text message from the user.
         * @param session the web socket session the message comes over on
         * @param message the message from the user
         * @throws Exception ignored
         */
        @Override
        protected void handleTextMessage(@Nonnull WebSocketSession session,
                        @Nonnull TextMessage message) throws Exception {
            super.handleTextMessage(session, message);
            this.session = session;

            JSONObject payload = new JSONObject(message.getPayload());

            if(payload.get("method").equals("cancel")) {
                cancelGoal.call(void.class,
                        Long.parseLong((String) payload.get("goalId")));
            } else if(payload.get("method").equals("suspend")) {
                submitGoal.call(long.class,
                        Factory.createPredicate(
                                "suspendGoal",
                                (String) payload.get("agent"),
                                (String) payload.get("goal")
                        ));
            } else if(payload.get("method").equals("resume")) {
                submitGoal.call(long.class,
                        Factory.createPredicate(
                                "resumeGoal",
                                (String) payload.get("agent"),
                                (String) payload.get("goal")
                        ));
            }
        }

        /**
         * Starts a periodic update after the connection is established to the GUI
         * client.
         * @param session the web socket session this endpoint communicates over
         */
        @Override
        public void afterConnectionEstablished(@Nonnull WebSocketSession session) {
            this.session = session;
            log.debug("Goal viewer connection established");

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
