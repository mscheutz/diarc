package edu.tufts.hrilab.simspeech;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.json.JSONObject;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.util.*;

/**
 * ChatEndpoint. Wraps a text web socket handler in a DIARC Component.
 */
@Component
public class ChatEndpointComponent extends DiarcComponent {
    private String[] robotNames;
    private HashMap<String, TRADEServiceInfo> robotInputs;
    private HashMap<String, TRADEServiceInfo> robotSetSpeakers;

    private final ChatHandler chatHandler;

    /**
     * Constructs the <code>ChatEndpoint</code>.
     */
    public ChatEndpointComponent() {
        this.chatHandler = new ChatHandler();
    }

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
        // Register for dialogue service
        try {
            TRADEServiceInfo sendMessageService = null;
            TRADEServiceInfo dialogueRegisterService = null;
            TRADE.registerAllServices(this, (String) null);
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            // Find send message service and dialogue register service
            for (TRADEServiceInfo service : availableServices) {
                if (service.serviceString.equals("sendMessage(edu.tufts.hrilab."
                        + "slug.common.Utterance)"))
                    sendMessageService = service;
                else if(service.serviceString.equals(
                        "registerForDialogueHistoryNotifications("
                        + "ai.thinkingrobots.trade.TRADEServiceInfo)"))
                    dialogueRegisterService = service;

                if(sendMessageService != null && dialogueRegisterService != null)
                    break;
            }

            if(sendMessageService == null || dialogueRegisterService == null)
                throw new NullPointerException("Could not find service");

            dialogueRegisterService.call(void.class, sendMessageService);

        } catch (TRADEException e) {
            log.error("Failed to register send message service");
        } catch (NullPointerException e) {
            log.error("Failed to register methods for dialogue history notifications");
        }

        // Find robot input services
        this.robotInputs = new HashMap<>();
        int mapped = 0;
        Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
        outer:
        for (TRADEServiceInfo service : availableServices) {
            if (service.serviceString.equals("setText(java.lang.String)")) {
                for(String robotName : this.robotNames) {
                    if(robotName.equals(
                            service.getGroups().iterator().next())) {
                        this.robotInputs.put(robotName, service);
                        mapped++;
                        continue outer;
                    }
                }
            }
        }
        if(mapped != this.robotNames.length)
            log.error("Failed to map all robot names to inputs");
        
        // Find robot set-speaker services
        this.robotSetSpeakers = new HashMap<>();
        mapped = 0;
        outer:
        for (TRADEServiceInfo service : availableServices) {
            if (service.serviceString.equals("setSpeaker(edu.tufts.hrilab.fol.Symbol)")) {
                for(String robotName : this.robotNames) {
                    if(robotName.equals(
                            service.getGroups().iterator().next())) {
                        this.robotSetSpeakers.put(robotName, service);
                        mapped++;
                        continue outer;
                    }
                }
            }
        }

        if(mapped != this.robotNames.length)
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

    /**
     * ChatHandler inner class. This implements the server.
     */
    public class ChatHandler extends TextWebSocketHandler {
        private WebSocketSession session;

        /**
         * Sends a message to the chat window.
         *
         * @param utterance what the robot is saying
         */
        public void sendMessage(Utterance utterance) {
            // Make sure the speaker is a robot
            boolean flag = false;
            for (String robotName : robotNames) {
                if (utterance.getSpeaker().toString().equals(robotName)) {
                    flag = true;
                    break;
                }
            }
            if (!flag) return;

            try {
                if (session != null) {
                    session.sendMessage(new TextMessage(
                            "{\"message\":\"" + utterance.getWordsAsString()
                                    + "\", \"sender\":\"" + utterance.getSpeaker()
                                    + "\", \"recipient\":\"" + utterance.getAddressee() + "\"}"
                    ));
                } else {
                    log.error("Could not send message: no active session");
                }
            } catch (Exception e) {
                log.error(e.getMessage());
            }
        }

        // +----------------------+
        // | TextWebSocketHandler |
        // +----------------------+

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
            // currently not needed
            String sender = request.getString("sender");
            String recipient = request.getString("recipient");

            TRADEServiceInfo setSpeaker = robotSetSpeakers.get(recipient);
            TRADEServiceInfo setText = robotInputs.get(recipient);
            setSpeaker.call(void.class, Factory.createSymbol(sender));
            setText.call(void.class, data);
        }
    }
}
