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
import org.springframework.beans.factory.NoSuchBeanDefinitionException;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.ConfigurableApplicationContext;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
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
@Configuration
@EnableWebSocket
@ComponentScan(basePackages= "edu.tufts.hrilab")
public class EndpointManagerComponent extends DiarcComponent
        implements WebSocketConfigurer {
    // +---------------------+
    // | WebSocketConfigurer |
    // +---------------------+

    @Autowired
    private ConfigurableApplicationContext applicationContext;

    @Value("${cors.origin}")
    private String corsOrigin;

    // Helper method to convert comma-separated String to an array
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    @Bean
    public MapGui mapGui(MapComponent mapComponent) {
        return new MapGui(mapComponent);
    }

    @Bean
    public MapComponent mapComponent() {
        String mapConfig = "-map_folder /home/hrilab/code/diarc-old/maps/elevator_lab_test/ -start_floor 1";
        if (mapConfig.isEmpty()) {
            // If the configuration string is empty, do not create the MapComponent
            return null;
        }
        MapComponent component = createInstance(MapComponent.class, mapConfig);
        applicationContext.getBeanFactory().registerSingleton("mapComponent", component);
        return component;
    }

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
                    .setAllowedOrigins(parseCorsOrigins());
            registry.addHandler(goalViewerHandler, "/goalViewer")
                    .setAllowedOrigins(parseCorsOrigins());
            registry.addHandler(goalManagerHandler, "/goalManager")
                    .setAllowedOrigins(parseCorsOrigins());
            // Conditionally add MapGui handler if MapComponent is available
            MapComponent mapComponent = applicationContext.getBean(MapComponent.class);
            if (mapComponent != null) {
                registry.addHandler(mapGui(mapComponent), "/map")
                        .setAllowedOrigins(parseCorsOrigins());
            }
        } catch (NoSuchBeanDefinitionException e) {
            log.info("MapComponent is not defined in the application context");
        } catch(TRADEException e) {
            log.error("Chat handler service call failed");
        }
    }
}
