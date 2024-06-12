package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.gui.ADBEWrapper;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.json.JSONArray;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.*;
import java.time.LocalDateTime;
import java.util.*;

/**
 * Wraps a text web socket handler in a DIARC component. Handles the server side
 * which sits between the <code>GoalManagerImpl</code> and the frontend goal
 * manager GUI.
 * @author Lucien Bao
 * @version 1.0
 */
@Component
public class GoalManagerEndpointComponent extends DiarcComponent {
    //==========================================================================
    // Constants
    //==========================================================================
    /**
     * Path to the folder containing all .asl files. Depending on method of
     * launch, might require prefix ".." or might not.
     */
    public final String ACTION_SCRIPT_PATH =
            "../core/src/main/resources/config/edu/tufts/hrilab/action/asl";

    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * The component's instance of the inner class.
     */
    private final GoalManagerHandler goalManagerHandler;

    /**
     * TRADE service to submit a goal.
     */
    private TRADEServiceInfo submitGoalService;

    /**
     * TRADE service to submit an action.
     */
    private TRADEServiceInfo submitActionService;

    /**
     * List of actions in the databse.
     */
    private final List<ActionDBEntry> actionList;

    /**
     * Utility object to write actions to an ASL file.
     */
    private final ActionScriptLanguageWriter aslWriter;

    //==========================================================================
    // Constructor
    //==========================================================================
    /**
     * Constructor. Instantiates the inner class.
     */
    public GoalManagerEndpointComponent() {
        this.goalManagerHandler = new GoalManagerHandler();

        initializeServices();

        actionList = new ArrayList<>();
        actionList.addAll(Database.getActionDB().getAllActions());
        actionList.sort(Comparator.comparing(e ->
                new ADBEWrapper(e).getActionSignature()));

        aslWriter = new ActionScriptLanguageWriter();
    }

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Find this instance's required TRADE services.
     */
    private void initializeServices() {
        try {
            submitActionService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("submitGoal")
                            .argTypes(Predicate.class)
                            .returnType(long.class)
            );
            submitGoalService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("submitGoal")
                            .argTypes(Goal.class)
                            .returnType(Goal.class)
            );
        } catch(TRADEException e) {
            log.error("Failed to get TRADE services for goal manager");
        }
    }

    /**
     * Crawls the ASL scripts directory recursively to find ASL files.
     * @return a JSON object containing the hierarchy of ASL files.
     */
    @Nonnull
    private JSONObject getAslFilesAsTree() {
        File root = new File(ACTION_SCRIPT_PATH);

        if (!root.isDirectory()) {
            log.error("Invalid action script path while getting tree");
            return new JSONObject(root.getName());
        }

        int counter = 0;
        JSONObject tree = new JSONObject();
        tree.put("name", root.getName())
                .put("id", "" + counter++);

        // Each object in the array is {file, file's JSONObject}
        Deque<Object[]> dfs = new ArrayDeque<>();
        dfs.push(new Object[] {root, tree});

        while (!dfs.isEmpty()) {
            Object[] current = dfs.pop();
            File file = (File) current[0];
            JSONObject object = (JSONObject) current[1];
            if (file.isDirectory()) {
                object.put("children", new JSONArray());
                // Shouldn't be null, if it's a directory...
                for (File childFile : Objects.requireNonNull(file.listFiles())) {
                    JSONObject childObject = new JSONObject()
                            .put("name", childFile.getName())
                            .put("id", "" + counter++);
                    object.append("children", childObject);
                    dfs.push(new Object[] {childFile, childObject});
                }
            }
        }

        return tree;
    }

    /**
     * Generates a filepath for the ASL writer to create a new file at.
     * @return a String representation of a file path
     */
    private String generateAslWriteFilePath() {
        LocalDateTime now = LocalDateTime.now();
        return ACTION_SCRIPT_PATH
                + "/custom/"
                + "gui-export-script-"
                + now
                .toString()
                // replace invalid or confusing chars
                .replace(':', '-')
                .replace('.', '-')
                + ".asl";
    }

    /**
     * Given a list of action IDs, export those actions to a file.
     * @param toExport list of action IDs
     */
    private void exportActionsAsFile(List<Object> toExport) {
        if(!new File(ACTION_SCRIPT_PATH + "/custom").mkdirs()) {
            log.error("Failed to ensure custom/ directory exists for asl files");
        }

        List<ActionDBEntry> toWrite = new ArrayList<>();
        for(Object o : toExport) {
            int i = ((Integer) o) - 1; // unshift index
            toWrite.add(actionList.get(i));
        }

        aslWriter.writeToFile(toWrite, generateAslWriteFilePath());

        goalManagerHandler.notifyExportSuccessful();
    }

    /**
     * Getter for the goal manager handler instance.
     * @return the goal manager handler
     */
    @TRADEService
    public GoalManagerHandler getGoalManagerHandler() {
        return this.goalManagerHandler;
    }

    //==========================================================================
    // Inner class | GoalManagerHandler
    //==========================================================================
    /**
     * Inner class which handles all server functions.
     */
    public class GoalManagerHandler extends TextWebSocketHandler {
        //======================================================================
        // Fields
        //======================================================================
        /**
         * Current session.
         */
        private WebSocketSession session;

        //======================================================================
        // Methods
        //======================================================================
        /**
         * Send a message to the client that the ASL export was successful.
         */
        private void notifyExportSuccessful() {
            try {
                if(session != null && session.isOpen()) {
                    session.sendMessage(new TextMessage(
                            new JSONObject()
                                    .put("export", "successful")
                                    .toString()
                    ));
                }
            } catch(IOException ignored) {}
        }

        /**
         * Called on receipt of custom-action message.
         * @param customAction the action string
         */
        private void submitCustomAction(String customAction) {
            try {
                submitActionService.call(
                        Goal.class,
                        Factory.createPredicate(customAction)
                );
            } catch (TRADEException e) {
                log.error("TRADEException occurred during invocation of custom action");
            }
        }

        /**
         * Called on receipt of form-action message.
         * @param arguments the array of arguments
         */
        private void submitFormAction(JSONArray arguments) {
            try {
                StringBuilder sb = new StringBuilder(arguments.getString(0))
                        .append('(');
                for (int i = 1; i < arguments.length(); i++) {
                    sb.append(arguments.getString(i))
                            .append(',');
                }
                sb.delete(sb.length() - 1, sb.length())
                        .append(')');
                submitActionService.call(
                        Goal.class,
                        Factory.createPredicate(sb.toString())
                );
            } catch(TRADEException e) {
                log.error("TRADEException occurred during invocation of form action");
            }
        }
        
        private void submitGoal(String agent, String goal) {
            try {
                submitGoalService.call(
                        Goal.class,
                        new Goal(Factory.createSymbol(agent),
                                Factory.createPredicate(goal))
                );
            } catch (TRADEException e) {
                log.error("TRADEException occurred during invocation of goal");
            }
        }

        //======================================================================
        // Implement methods | TextWebSockethandler
        //======================================================================
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
            JSONObject payload = new JSONObject(message.getPayload());

            switch ((String) payload.get("type")) {
                case "custom" -> submitCustomAction(
                        payload.getJSONObject("formData").getString("custom"));
                case "form" -> submitFormAction(payload.getJSONArray("formData"));
                case "goal" -> submitGoal(
                        payload.getJSONObject("formData").getString("agent"),
                        payload.getJSONObject("formData").getString("goal")
                );
                case "export" -> exportActionsAsFile(
                        payload.getJSONArray("selected").toList());
            }
        }

        /**
         * Starts a periodic update after the connection is established to the GUI
         * client.
         * @param session the web socket session this endpoint communicates over
         */
        @Override
        public void afterConnectionEstablished(@Nonnull WebSocketSession session)
                throws Exception {
            this.session = session;
            log.debug("Goal manager connection established");

            JSONObject message = new JSONObject();

            JSONArray actions = new JSONArray();
            for(int i = 0; i < actionList.size(); i++) {
                actions.put(
                        new JSONObject()
                                .put("name", new ADBEWrapper(actionList.get(i))
                                        .getActionSignature())
                                .put("id", i+1) // id 0 is taken by the root
                );
            }
            message.put("actions", actions);

            JSONObject files = getAslFilesAsTree();
            message.put("files", files);

            session.sendMessage(new TextMessage(message.toString()));
        }

        /**
         * Called when the connection is closed.
         * @param session the session that was terminated
         * @param status close status
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
