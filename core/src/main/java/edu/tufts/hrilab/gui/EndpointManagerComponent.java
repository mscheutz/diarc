package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalManagerEndpointComponent;
import edu.tufts.hrilab.action.GoalViewerEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
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
    // +---------------------+

    /**
     * Add endpoints and allow the client to access them.
     * @param registry registry object that configures request mappings
     */
    @Override
    public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
        try {
            WebSocketHandler chatHandler = null;
            WebSocketHandler goalViewerHandler = null;
            WebSocketHandler goalManagerHandler = null;
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                switch (service.serviceString) {
                    case "getChatHandler()" -> chatHandler = service.call(ChatEndpointComponent.ChatHandler.class);
                    case "getGoalViewerHandler()" ->
                            goalViewerHandler = service.call(GoalViewerEndpointComponent.GoalViewerHandler.class);
                    case "getGoalManagerHandler()" ->
                            goalManagerHandler = service.call(GoalManagerEndpointComponent.GoalManagerHandler.class);
                }

                if(chatHandler != null && goalViewerHandler != null
                && goalManagerHandler != null)
                    break;
            }

            if(chatHandler == null || goalViewerHandler == null
            || goalManagerHandler == null)
                throw new NullPointerException("Failed to find handler of "
                + "chat, goal viewer, or goal manager");

            registry.addHandler(chatHandler, "/chat")
                    .setAllowedOrigins("http://localhost:3000");
            registry.addHandler(goalViewerHandler, "/goalViewer")
                    .setAllowedOrigins("http://localhost:3000");
            registry.addHandler(goalManagerHandler, "/goalManager")
                    .setAllowedOrigins("http://localhost:3000");
        } catch(TRADEException e) {
            log.error("Chat handler service call failed");
        }
    }
}
