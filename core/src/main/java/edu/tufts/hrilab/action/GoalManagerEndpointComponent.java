package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.gui.ADBEWrapper;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.util.IdGenerator;
import org.json.JSONArray;
import org.json.JSONObject;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class GoalManagerEndpointComponent extends DiarcComponent {
    private final GoalManagerHandler goalManagerHandler;
//    public final String ACTION_SCRIPT_PATH = "core/src/main/resources/config/edu/tufts/hrilab/action/asl";
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
        private final HashMap<Integer, File> aslFileMap;
        private TRADEServiceInfo submitGoalService;
        private TRADEServiceInfo submitActionService;

        private final List<ActionDBEntry> actionList;

        private final ActionScriptLanguageWriter aslWriter;

        /**
         * Constructs this GoalEndpoint.
         */
        public GoalManagerHandler() {
            aslFileMap = new HashMap<>();
            try {
                initializeServices();
            } catch(TRADEException e) {
                log.error("Failed to get TRADE services");
                log.error(e.getMessage());
            } catch(NullPointerException e) {
                log.error(e.getMessage());
            }

            actionList = new ArrayList<>();
            actionList.addAll(Database.getActionDB().getAllActions());
            actionList.sort(Comparator.comparing(e ->
                    new ADBEWrapper(e).getActionSignature()));

            aslWriter = new ActionScriptLanguageWriter();
        }

        /**
         * Find this instance's required TRADE services.
         */
        private void initializeServices() throws TRADEException {
            submitActionService = null;
            submitGoalService = null;
            // For some reason there are multiple submitGoal() functions that
            // will not be differentiated via TRADEServiceConstraints
            // ... which should not be happening but anyway...
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                if (service.serviceString.equals("submitGoal(edu.tufts.hrilab.fol.Predicate)")) {
                    submitActionService = service;
                }
                else if(service.serviceString.equals("submitGoal(edu.tufts.hrilab.action.goal.Goal)")) {
                    submitGoalService = service;
                }
                if(submitGoalService != null && submitActionService != null)
                    break;
            }
            if(submitActionService == null)
                throw new NullPointerException("Could not find submitActionService");
            if(submitGoalService == null)
                throw new NullPointerException("Could not find submitGoalService");
        }

        /**
         * Convert the declaration of an action into an action signature.
         * @param line the line containing the declaration
         * @return the action signature
         */
        private String getActionString(String line) {
            StringBuilder sb = new StringBuilder();
            Pattern name = Pattern.compile("= *(\\w*)(\\[.*)? *\\(");
            Matcher m = name.matcher(line);
            if(m.find()) {
                sb.append(m.group(1))
                        .append("(?actor, ");
            } else {
                throw new IllegalStateException("Could not find action name");
            }

            Pattern arguments = Pattern.compile("(\\?\\w*)");
            m = arguments.matcher(line.substring(m.end()));
            while(m.find()) {
                sb.append(m.group())
                        .append(", ");
            }

            sb.delete(sb.length() - 2, sb.length())
                    .append(')');
            return sb.toString();
        }

        /**
         * Crawls the ASL scripts directory recursively to find actions.
         * @return an array containing a list of action signatures.
         */
        private String[] getAslActions() {
            String[] paths = getAslFilesAsArray();
            ArrayList<String> actions = new ArrayList<>();

            Pattern regex = Pattern.compile("\\(.*\\)\\s?=\\s?.*\\(.*\\)\\s\\{");

            try {
                for (String path : paths) {
                    BufferedReader br = new BufferedReader(new FileReader(path));
                    String line = br.readLine();
                    while(line != null) {

                        Matcher matcher = regex.matcher(line);
                        if(matcher.matches()) {
                            actions.add(getActionString(line));
                        }

                        line = br.readLine();
                    }
                }
            } catch (IOException ignored) {}

            // Java hates me if I use toArray()
            String[] result = new String[actions.size()];
            for(int i = 0; i < actions.size(); i++)
                result[i] = actions.get(i);
            return result;
        }

        /**
         * Crawls the ASL scripts directory recursively to find ASL files.
         * @return an array containing a list of ASL files.
         */
        @Nonnull
        private String[] getAslFilesAsArray() {
            File root = new File(ACTION_SCRIPT_PATH);

            if (!root.isDirectory()) {
                log.error("Invalid action script path while getting array");
                return new String[] {};
            }

            ArrayList<String> arrayList = new ArrayList<>();
            Deque<File> dfs = new ArrayDeque<>();
            dfs.push(root);

            while (!dfs.isEmpty()) {
                File file = dfs.pop();
                if (file.isDirectory()) {
                    // Shouldn't be null, if it's a directory...
                    for (File f : Objects.requireNonNull(file.listFiles())) {
                        dfs.push(f);
                    }
                } else if (file.isFile()) {
                    arrayList.add(file.getPath());
                }
            }

            // Java hates me if I use toArray()
            String[] result = new String[arrayList.size()];
            for(int i = 0; i < arrayList.size(); i++)
                result[i] = arrayList.get(i);
            return result;
        }

        /**
         * Crawls the ASL scripts directory recursively to find ASL files.
         * @return an object containing the hierarchy of ASL files.
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
            aslFileMap.put(counter, root);
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
                        aslFileMap.put(counter, childFile);
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
                        .replace(':', '-')
                        .replace('.', '-')
                    + ".asl";
        }

        /**
         * Given a list of action IDs, export those actions to a file.
         * @param toExport list of action IDs
         */
        private void exportActionsAsFile(List<Object> toExport) {
            // Make sure the custom/ directory exists
            new File(ACTION_SCRIPT_PATH + "/custom").mkdirs();

            List<ActionDBEntry> toWrite = new ArrayList<>();
            for(Object o : toExport) {
                int i = ((Integer) o) - 1; // unshift index
                toWrite.add(actionList.get(i));
            }

            aslWriter.writeToFile(toWrite, generateAslWriteFilePath());
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
            JSONObject payload = new JSONObject(message.getPayload());

            if(payload.has("fileId")) {
                String request = (String) new JSONObject(message.getPayload())
                        .get("fileId");
                int requestId = Integer.parseInt(request);
                File file = aslFileMap.get(requestId);
                String response = Files.readString(Path.of(file.getPath()));

                JSONObject responseObject = new JSONObject();
                responseObject.put("contents", response);
                session.sendMessage(new TextMessage(responseObject.toString()));
            } else if(payload.has("form")) {
                // Handle goal submission
                if(payload.get("form").equals("goal")) {
                    String agent = payload.getJSONObject("formData")
                                          .getString("agent");
                    String goal = payload.getJSONObject("formData")
                                         .getString("goal");
                    submitGoalService.call(
                        Goal.class,
                        new Goal(Factory.createSymbol(agent),
                                Factory.createPredicate(goal))
                    );
                }
                // Handle custom action submission
                else if(payload.get("form").equals("custom")){
                    String customAction = payload.getJSONObject("formData")
                                                 .getString("custom");
                    try {
                        submitActionService.call(
                                Goal.class,
                                Factory.createPredicate(customAction)
                        );
                    } catch (TRADEException e) {
                        log.error("TRADEException occured in invocation of "
                                + "custom action");
                    }
                }
                // Handle generated action submission
                else {
                    JSONArray arguments = payload.getJSONArray("formData");
                    StringBuilder sb = new StringBuilder(arguments.getString(0))
                            .append('(');
                    for(int i = 1; i < arguments.length(); i++) {
                        sb.append(arguments.getString(i))
                                .append(',');
                    }
                    sb.delete(sb.length() - 1, sb.length())
                            .append(')');
                    submitActionService.call(
                        Goal.class,
                        Factory.createPredicate(sb.toString())
                    );
                }
            }
            // Handle export ASL actions
            else if(payload.has("selected")) {
                exportActionsAsFile(payload.getJSONArray("selected")
                        .toList());
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
    }
}
