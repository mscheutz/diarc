package edu.tufts.hrilab.simspeech;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.IOException;
import java.util.*;

/**
 * Wraps a text web socket handler in a DIARC component. Handles the server side
 * which sits between <code>SimSpeechRecognitionComponent</code>s and the
 * frontend chat GUI.
 * @author Lucien Bao
 * @version 1.0
 */
@Component
public class ChatEndpointComponent extends DiarcComponent {
    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * List of robot agents.
     */
    private String[] robotNames;
    /**
     * Maps each robot name to a TRADE service to make that robot receive text
     * messages through their <code>SimSpeechRecognitionComponent</code>s.
     */
    private HashMap<String, TRADEServiceInfo> robotInputs;
    /**
     * Maps each robot name to a TRADE service to set the current speaker for
     * that robot. This allows different people to talk to the robots.
     */
    private HashMap<String, TRADEServiceInfo> robotSetSpeakers;

    /**
     * The component's instance of the inner class.
     */
    private final ChatHandler chatHandler;

    //==========================================================================
    // Constructor
    //==========================================================================
    /**
     * Constructor. Instantiates the inner class.
     */
    public ChatEndpointComponent() {
        this.chatHandler = new ChatHandler();
    }

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Parse runtime arguments after construction.
     * @param cmdLine the list of arguments
     */
    @Override
    protected void parseArgs(CommandLine cmdLine) {
        // This is a required option
        this.robotNames = cmdLine.getOptionValues("names");
    }

    /**
     * Provide a list of arguments available in the command line.
     * @return the list of arguments
     */
    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("n")
                        .longOpt("names")
                        .desc("list of robot names, separated by spaces")
                        .valueSeparator(' ')
                        .numberOfArgs(Option.UNLIMITED_VALUES)
                        .required(true)
                        .build());
        return options;
    }

    /**
     * Complete initialization after construction and parsing of command line
     * arguments.
     */
    @Override
    protected void init() {
        registerForDialogueService();
        mapRobotInputs();
        mapRobotSetSpeakers();
    }

    /**
     * Registers this component's <code>sendMessage()</code> TRADE service
     * and links it to the <code>DialogueComponent</code>'s notification
     * registry so that it will be called when new <code>Utterances</code> are
     * added.
     */
    private void registerForDialogueService() {
        try {
            // We must register this component's `sendMessage()` service first
            TRADE.registerAllServices(this, (String) null);

            TRADEServiceInfo sendMessageService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("sendMessage")
                            .argTypes(Utterance.class)
            );
            TRADEServiceInfo dialogueRegisterService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .name("registerForDialogueHistoryNotifications")
                            .argTypes(TRADEServiceInfo.class)
            );

            dialogueRegisterService.call(void.class, sendMessageService);
        } catch (TRADEException e) {
            log.error("Failed to register send message service");
        }
    }

    /**
     * Find the <code>setText()</code> TRADE services for each robot and store
     * them for later use.
     */
    private void mapRobotInputs() {
        robotInputs = new HashMap<>();

        try {
            for (String robotName : robotNames) {
                TRADEServiceInfo service = TRADE.getAvailableService(
                        new TRADEServiceConstraints()
                                .name("setText")
                                .argTypes(String.class)
                                .inGroups(robotName)
                );
                robotInputs.put(robotName, service);
            }
        } catch(TRADEException e) {
            log.error("Failed to find robot input TRADE service");
        }

        if(robotInputs.size() != robotNames.length)
            log.error("Failed to map all robot names to inputs");
    }

    /**
     * Find the <code>setSpeaker()</code> TRADE services for each robot and
     * store them for later use.
     */
    private void mapRobotSetSpeakers() {
        robotSetSpeakers = new HashMap<>();

        try {
            for(String robotName : robotNames) {
                TRADEServiceInfo service = TRADE.getAvailableService(
                        new TRADEServiceConstraints()
                                .name("setSpeaker")
                                .argTypes(Symbol.class)
                                .inGroups(robotName)
                );
                robotSetSpeakers.put(robotName, service);
            }
        } catch (TRADEException e) {
            log.error("Failed to find robot setSpeaker TRADE service");
        }

        if(robotSetSpeakers.size() != robotNames.length)
            log.error("Failed to find setSpeaker TRADE services");
    }

    /**
     * Getter for the chat handler instance.
     * @return the chat handler
     */
    @TRADEService
    public ChatHandler getChatHandler() {
        return chatHandler;
    }

    /**
     * Sends a message to the chat window.
     *
     * @param utterance what the robot is saying
     */
    @TRADEService
    public void sendMessage(Utterance utterance) {
        this.chatHandler.sendMessage(utterance);
    }

    //==========================================================================
    // Inner class | ChatHandler
    //==========================================================================
    /**
     * Inner class which handles all server functions.
     */
    public class ChatHandler extends TextWebSocketHandler {
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
         * Check if the speaker of the given utterance is a robot tracked by
         * this component.
         * @param utterance utterance to check
         * @return true if the given utterance was spoken by on of this
         * component's tracked robots
         */
        private boolean isSpeakerRobot(Utterance utterance) {
            for(String robotName : robotNames) {
                if(utterance.getSpeaker().toString().equals(robotName)) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Sends a message to the chat window.
         *
         * @param utterance what the robot is saying
         */
        public void sendMessage(Utterance utterance) {
            if (!isSpeakerRobot(utterance)) return;

            try {
                if (session != null) {
                    session.sendMessage(new TextMessage(
                            new JSONObject()
                                    .put("message", utterance.getWordsAsString())
                                    .put("sender", utterance.getSpeaker())
                                    .put("recipient", utterance.getAddressee())
                                    .toString()
                    ));
                } else {
                    log.error("Could not send message: no active session");
                }
            } catch (IOException e) {
                log.error("Failed to send message to chat");
            }
        }

        //======================================================================
        // Implementing methods | TextWebSocketHandler
        //======================================================================
        /**
         * Called after being connected to the client.
         * @param session the web socket session the message comes over on
         * @throws Exception ignored
         */
        @Override
        public void afterConnectionEstablished(@Nonnull WebSocketSession session)
                throws Exception {
            super.afterConnectionEstablished(session);

            JSONObject setupMessage = new JSONObject();
            setupMessage.put("names", Arrays.toString(robotNames));
            session.sendMessage(new TextMessage(setupMessage.toString()));
        }

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
            this.session = session;
            JSONObject request = new JSONObject(message.getPayload());

            String data = request.getString("message");
            String sender = request.getString("sender");
            String recipient = request.getString("recipient");

            TRADEServiceInfo setSpeaker = robotSetSpeakers.get(recipient);
            setSpeaker.call(void.class, Factory.createSymbol(sender));

            TRADEServiceInfo setText = robotInputs.get(recipient);
            setText.call(void.class, data);
        }
    }
}
