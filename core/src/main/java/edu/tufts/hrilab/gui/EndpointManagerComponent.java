package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import org.apache.commons.cli.CommandLine;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;
import java.util.Collection;

/**
 * EndpointManagerComponent. A DIARC component that sets up and maintains
 * a websocket server that communicates to the browser-based GUI.
 */
@Component
@EnableWebSocket
public class EndpointManagerComponent extends DiarcComponent
        implements WebSocketConfigurer {
    // +---------------------+
    // | WebSocketConfigurer |
    // + --------------------+

    @Value("${cors.origin}")
    private String corsOrigin;

    // Convert the comma-separated String to an array
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    /**
     * Add endpoints and allow the client to access them.
     * @param registry registry object that configures request mappings
     */
    @Override
    public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
        try {
            WebSocketHandler chatHandler = null;
            WebSocketHandler goalHandler = null;
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                if (service.serviceString.equals("getChatHandler()"))
                    chatHandler = service.call(ChatEndpointComponent.ChatHandler.class);
                else if(service.serviceString.equals("getGoalHandler()"))
                    goalHandler = service.call(GoalEndpointComponent.GoalHandler.class);

                if(chatHandler != null && goalHandler != null)
                    break;
            }

            if(chatHandler == null || goalHandler == null)
                throw new NullPointerException("Failed to find handler of chat or goal");

            registry.addHandler(chatHandler, "/chat")
                    .setAllowedOrigins(parseCorsOrigins());
            registry.addHandler(goalHandler, "/goal")
                    .setAllowedOrigins(parseCorsOrigins());
        } catch(TRADEException e) {
            log.error("Chat handler service call failed");
        }
    }
}
