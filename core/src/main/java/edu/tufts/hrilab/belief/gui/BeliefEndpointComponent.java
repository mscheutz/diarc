package edu.tufts.hrilab.belief.gui;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.IOException;
import java.util.*;

/**
 * Wraps a text web socket handler in a DIARC component. Handles the server side
 * which sits between the <code>BeliefComponent</code> and the frontend belief
 * viewer GUI.
 * @author Lucien Bao
 * @version 1.0
 */
@Component
public class BeliefEndpointComponent extends DiarcComponent {
    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * This component's instance of the inner class.
     */
    private final BeliefHandler beliefHandler;

    /**
     * A reference to the <code>BeliefComponent</code>'s prover.
     */
    private Prover prover;

    /**
     * An array of previous beliefs, refreshed on each update. Used to find deltas
     * for asserted and retracted beliefs.
     */
    private String[] previousBeliefs;

    /**
     * An unformatted array beliefs used as a buffer.
     */
    private String[] unformattedBeliefs;

    /**
     * A list of current beliefs.
     */
    private final List<String> currentBeliefs;

    /**
     * A list holding the timeline of belief changes.
     */
    private final List<String> timelineBeliefs;

    //==========================================================================
    // Constructor
    //==========================================================================
    /**
     * Constructor. Instantiates the inner class.
     */
    public BeliefEndpointComponent() {
        this.beliefHandler = new BeliefHandler();
        this.previousBeliefs = new String[] {};
        this.unformattedBeliefs = new String[] {};
        this.currentBeliefs = new LinkedList<>();
        this.timelineBeliefs = new LinkedList<>();
    }

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Finish initialization after construction and DIARC component loading.
     */
    @Override
    protected void init() {
        try {
            TRADEServiceInfo getProverService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .returnType(Prover.class)
                            .name("getNotificationProver")
                            .argTypes()
            );
            this.prover = getProverService.call(Prover.class);

            String allBeliefs = prover.getTheory();
            previousBeliefs = allBeliefs.split(".\n\n");
            unformattedBeliefs = previousBeliefs;
            formatList(currentBeliefs);
        } catch (TRADEException e) {
            log.error("Failed to register getProver service");
        }
    }

    /**
     * Reads unformatted beliefs from the buffer into the specified list.
     * @param list list to <em>prepend</em> formatted beliefs into.
     */
    private void formatList(List<String> list) {
        for(String s : unformattedBeliefs) {
            // Empty belief -> add blank to front of list
            if(s.isEmpty()) {
                list.add(0, "\n");
            }
            // Full belief -> make sure not just a close parenthesis
            else if(s.replace(" ", "").length() > 1) {
                list.add(0, s);
            }
        }
    }

    /**
     * Given an array of beliefs, updates the timeline based on the delta
     * between this array and the currently-held belief array.
     * @param beliefs incoming array of beliefs
     */
    private void updateTimelineBeliefs(String[] beliefs) {
        List<String> toUpdate = new ArrayList<>();
        if(beliefs != previousBeliefs) {
            for(String s : beliefs) {
                if(!Arrays.asList(previousBeliefs).contains(s)) {
                    toUpdate.add("assert:" + s);
                }
            }
            for(String s : previousBeliefs) {
                if(!Arrays.asList(beliefs).contains(s)) {
                    toUpdate.add("retract:" + s);
                }
            }
        }

        String[] updatedList = new String[toUpdate.size()];
        for(int i = 0; i < toUpdate.size(); i++) {
            updatedList[i] = toUpdate.get(i);
        }
        unformattedBeliefs = updatedList;
        formatList(timelineBeliefs);
    }

    /**
     * Given an array of beliefs, copies over the array to the list of current
     * beliefs.
     * @param beliefs incoming array of beliefs
     */
    private void updateCurrentBeliefs(String[] beliefs) {
        currentBeliefs.clear();
        unformattedBeliefs = beliefs;
        formatList(currentBeliefs);
    }

    /**
     * Getter for this component's inner class instance.
     * @return this component's instance of the web socket handler.
     */
    @TRADEService
    public BeliefHandler getBeliefHandler() {
        return this.beliefHandler;
    }

    //==========================================================================
    // Inner class | BeliefHandler
    //==========================================================================
    /**
     * Inner class which handles all server functions.
     */
    public class BeliefHandler extends TextWebSocketHandler {
        //==========================================================================
        // Constants
        //==========================================================================
        /**
         * Timer update period, in milliseconds.
         */
        public static final int UPDATE_PERIOD = 1000;

        //==========================================================================
        // Fields
        //==========================================================================
        /**
         * Timer that periodically updates the client with belief info.
         */
        private Timer updateTimer;

        /**
         * Current session.
         */
        private WebSocketSession session;

        //==========================================================================
        // Methods
        //==========================================================================
        /**
         * Updates the user about system beliefs.
         */
        public void updateBeliefs() {
            try {
                if(session != null) {
                    // Adapted from refreshGui() in BeliefGui
                    String allBeliefs = prover.getTheory();
                    String[] allBeliefsArray = allBeliefs.split(".\n\n");

                    updateTimelineBeliefs(allBeliefsArray);
                    updateCurrentBeliefs(allBeliefsArray);
                    previousBeliefs = allBeliefsArray;

                    JSONObject update = new JSONObject()
                            .put("current", currentBeliefs)
                            .put("timeline", timelineBeliefs);

                    session.sendMessage(new TextMessage(update.toString()));
                } else {
                    log.debug("Could not send belief update: no active session");
                    updateTimer.cancel();
                }
            } catch(Exception e) {
                log.error(e.getMessage());
            }
        }

        /**
         * Handle a user query.
         * @param input query input
         */
        private void handleQuery(String input) {
            String belief = prover.queryBelief(Factory.createPredicate(input))
                    .toString();
            // String concatenation to convert boolean to String
            // "[{}]" == true, "[]" == false
            String result = belief.equals("[{}]") + "";

            try {
                this.session.sendMessage(
                        new TextMessage(
                                new JSONObject()
                                        .put("result", result)
                                        .toString())
                );
            } catch (IOException ioe) {
                log.error("Could not send query result message");
            }
        }

        /**
         * Handle a user assertion.
         * @param input assertion input
         */
        private void handleAssertion(String input) {
            handleQuery("assert(" + input + ")");
        }

        /**
         * Handle a user retraction.
         * @param input retraction input
         */
        private void handleRetraction(String input) {
            handleQuery("retract(" + input + ")");
        }

        //==========================================================================
        // Implementing methods | TextWebSocketHandler
        //==========================================================================
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
            String method = payload.getString("method");
            String input = payload.getString("input");

            switch(method) {
                case "query" -> handleQuery(input);
                case "assert" -> handleAssertion(input);
                case "retract" -> handleRetraction(input);
            }
        }

        /**
         * Starts a periodic update after the connection is established to the
         * GUI client.
         * @param session the web socket session established
         * @throws Exception ignored
         */
        @Override
        public void afterConnectionEstablished(@Nonnull WebSocketSession session)
                throws Exception {
            super.afterConnectionEstablished(session);
            this.session = session;
            log.debug("Belief viewer connection established");

            updateTimer = new Timer();
            updateTimer.scheduleAtFixedRate(new TimerTask() {
                @Override
                public void run() {
                    updateBeliefs();
                }
            }, 0, UPDATE_PERIOD);
        }

        /**
         * Performs cleanup when the connection is closed.
         * @param session the web socket session this endpoint communicates over
         * @throws Exception ignored
         */
        @Override
        public void afterConnectionClosed(@Nonnull WebSocketSession session,
                                          @Nonnull CloseStatus status) throws Exception {
            super.afterConnectionClosed(session, status);
            this.session = null;
            updateTimer.cancel();
            log.debug("Belief viewer connection closed");
        }
    }
}
