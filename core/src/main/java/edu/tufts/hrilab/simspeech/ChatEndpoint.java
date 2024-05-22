package edu.tufts.hrilab.simspeech;

import ai.thinkingrobots.trade.*;
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

    final private SimSpeechRecognitionComponent recognitionComponent;
    final private DialogueComponent dialogueComponent;
    final private String robotName;

    /**
     * Constructs the <code>ChatEndpoint</code> given simulated speech
     * recognition and production components.
     * @param recognitionComponent simulated speech recognition component
     */
    public ChatEndpoint(SimSpeechRecognitionComponent recognitionComponent,
                        DialogueComponent dialogueComponent,
                        String robotName) {
        this.recognitionComponent = recognitionComponent;
        this.dialogueComponent = dialogueComponent;
        this.robotName = robotName;

        try {
            TRADE.registerAllServices(this, (String) null);
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                if (service.serviceString.equals("sendMessage(edu.tufts.hrilab."
                        + "slug.common.Utterance)")) {
                    this.dialogueComponent.registerForDialogueHistoryNotifications(service);
                }
            }
        } catch(TRADEException e) {
            log.error("Failed to register methods for dialogue history notifications");
        }
    }

    @Override
    protected void handleTextMessage(@Nonnull WebSocketSession session,
                                     @Nonnull TextMessage message) throws Exception {
        JSONObject request = new JSONObject(message.getPayload());

        System.out.println(request);
        // TODO: implement reception of message → pass to recognition component
    }

    /**
     * Sends a message to the chat window.
     * @param utterance what the robot is saying.
     */
    @TRADEService
    public void sendMessage(Utterance utterance) {
        // The robot is not the one speaking
        if(!utterance.getSpeaker().toString().equals(this.robotName))
            return;

        try {
            // TODO: post message
            System.out.println("I was told to say: " + utterance.getWordsAsString());
        } catch (Exception e) {
            log.error(e.getMessage());
        }
    }
}
