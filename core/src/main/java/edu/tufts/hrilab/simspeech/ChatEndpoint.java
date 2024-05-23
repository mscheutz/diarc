package edu.tufts.hrilab.simspeech;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import org.json.JSONObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.util.Collection;

@Component
public class ChatEndpoint extends TextWebSocketHandler {
    private static final Logger log =
            LoggerFactory.getLogger(ChatEndpoint.class);

    final private SimSpeechRecognitionComponent[] recognitions;
    final private String[] robotNames;

    private WebSocketSession session;

    /**
     * Constructs the <code>ChatEndpoint</code> given simulated speech
     * recognition and production components.
     * @param recognitions simulated speech recognition component
     * @param dialogue dialogue component to register for utterance notifications.
     */
    public ChatEndpoint(SimSpeechRecognitionComponent[] recognitions,
                        DialogueComponent dialogue,
                        String[] robotNames) {
        this.recognitions = recognitions;
        this.robotNames = robotNames;

        try {
            TRADE.registerAllServices(this, (String) null);
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                if (service.serviceString.equals("sendMessage(edu.tufts.hrilab."
                        + "slug.common.Utterance)")) {
                    dialogue.registerForDialogueHistoryNotifications(service);
                }
            }
        } catch(TRADEException e) {
            log.error("Failed to register methods for dialogue history notifications");
        }
    }

    /**
     * Sends a message to the chat window.
     * @param utterance what the robot is saying.
     */
    @TRADEService
    public void sendMessage(Utterance utterance) {
        // Make sure a robot is speaking
        boolean flag = false;
        for(String robotName : robotNames) {
            if (utterance.getSpeaker().toString().equals(robotName)) {
                flag = true;
                break;
            }
        }
        if(!flag) return;

        try {
            if(session != null) {
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

    //======================
    // TextWebSocketHandler
    //======================

    /**
     * Handle a text message from the user.
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

        // Find the SimSpeechRecognitionComponent that matches speaker/listener
        // with sender/recipient of this message
        for(SimSpeechRecognitionComponent recognition : recognitions) {
            if(recognition.getListener().toString().equals(recipient)) {
                recognition.setSpeaker(new Symbol(sender));
                recognition.setText(data);
                break;
            }
        }
        log.error("Invalid incoming message: no SimSpeechRecognitionComponent"
                + "matches specified recipient");
    }
}
