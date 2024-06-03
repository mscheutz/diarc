package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.json.JSONArray;
import org.json.JSONObject;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.File;
import java.util.*;

public class GoalManagerEndpointComponent extends DiarcComponent {
    private final GoalManagerHandler goalManagerHandler;
    public final String ACTION_SCRIPT_PATH = "../core/src/main/resources/config/edu/tufts/hrilab/action/asl";

    /**
     * Constructor.
     */
    public GoalManagerEndpointComponent() {
        this.goalManagerHandler = new GoalManagerHandler();
    }

    /**
     * Getter for the goal manager handler instance.
     * @return the goal manager handler
     */
    @TRADEService
    public GoalManagerHandler getGoalManagerHandler() {
        return this.goalManagerHandler;
    }

    /**
     * GoalManagerHandler inner class. This implements the server.
     */
    public class GoalManagerHandler extends TextWebSocketHandler {
        private WebSocketSession session;

        /**
         * Constructs this GoalEndpoint.
         */
        public GoalManagerHandler() {
        }

        /**
         * Crawls the ASL scripts directory recursively to find actions.
         * @return an array containing a list of action signatures.
         */
        private String[] getActions() {return null;}

        /**
         * Crawls the ASL scripts directory recursively to find ASL files.
         * @return an array containing a list of ASL files.
         */
        private String[] getFiles() {
            File root = new File(ACTION_SCRIPT_PATH);

            if(!root.isDirectory()) {
                log.error("Invalid action script path");
                return null;
            }

            ArrayList<String> arrayList = new ArrayList<>();
            Deque<File> dfs = new ArrayDeque<>();
            dfs.push(root);

            while(!dfs.isEmpty()) {
                File file = dfs.pop();
                if(file.isDirectory()) {
                    // Shouldn't be null, if it's a directory...
                    for(File f : Objects.requireNonNull(file.listFiles())) {
                        dfs.push(f);
                    }
                } else if(file.isFile()) {
                    arrayList.add(file.getName());
                }
            }

            //noinspection DataFlowIssue
            return (String[]) arrayList.toArray();
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
        public void afterConnectionEstablished(WebSocketSession session) throws Exception {
            this.session = session;
            log.info("Connection established");

            JSONObject message = new JSONObject();

            // TODO
            getActions();

            session.sendMessage(new TextMessage(message.toString()));
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
        }
    }
}
