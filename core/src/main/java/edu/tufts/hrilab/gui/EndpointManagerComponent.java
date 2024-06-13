package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalManagerEndpointComponent;
import edu.tufts.hrilab.action.GoalViewerEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.map.MapGui;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;
import java.util.Collection;

/**
 * A DIARC component that sets up and maintains a websocket server that
 * communicates to the browser-based GUI. Depending on command-line arguments,
 * will initiate and expose the various endpoints.
 * @author Lucien Bao, Hengxu Li
 * @version 1.0
 */
@Configuration
@EnableWebSocket
@ComponentScan(basePackages="edu.tufts.hrilab")
public class EndpointManagerComponent extends DiarcComponent
        implements WebSocketConfigurer {
    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * The base URL for the server.
     */
    @Value("${app.base-url}")
    private String baseUrl;

    /**
     * Allowed cross-origin URLs, separated by commas.
     */
    @Value("${cors.origin}")
    private String corsOrigin;

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Helper method to convert the cross-origin URL string to a string array.
     * @return a string array of allowed cross origins.
     */
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    //==========================================================================
    // Implement methods | WebSocketConfigurer
    //==========================================================================
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
            MapComponent mapComponent = null;
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                switch (service.serviceString) {
                    case "getChatHandler()" ->
                            chatHandler = service.call(ChatEndpointComponent.ChatHandler.class);
                    case "getGoalViewerHandler()" ->
                            goalViewerHandler = service.call(GoalViewerEndpointComponent.GoalViewerHandler.class);
                    case "getGoalManagerHandler()" ->
                            goalManagerHandler = service.call(GoalManagerEndpointComponent.GoalManagerHandler.class);
                    case "getMapComponent()" ->
                            mapComponent = service.call(MapComponent.class);
                }

                if(chatHandler != null && goalViewerHandler != null
                && goalManagerHandler != null && mapComponent != null)
                    break;
            }

            if(chatHandler != null)
                registry.addHandler(chatHandler, "/chat")
                        .setAllowedOrigins(parseCorsOrigins());
            if(goalViewerHandler != null)
                registry.addHandler(goalViewerHandler, "/goalViewer")
                        .setAllowedOrigins(parseCorsOrigins());
            if(goalManagerHandler != null)
                registry.addHandler(goalManagerHandler, "/goalManager")
                        .setAllowedOrigins(parseCorsOrigins());
            if (mapComponent != null)
                registry.addHandler(new MapGui(baseUrl, mapComponent), "/map")
                        .setAllowedOrigins(parseCorsOrigins());
        } catch(TRADEException e) {
            log.error("Endpoint get service call failed");
        }
    }
}
