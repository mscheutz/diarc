package edu.tufts.hrilab.gui;

import edu.tufts.hrilab.action.GoalEndpoint;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.simspeech.ChatEndpoint;
import org.apache.commons.cli.CommandLine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;

/**
 * EndpointManagerComponent. A DIARC component that sets up and maintains
 * a websocket server that communicates to the browser-based GUI.
 */
@Component
@EnableWebSocket
public class EndpointManagerComponent extends DiarcComponent
        implements WebSocketConfigurer {

    private static ChatEndpoint chatEndpoint;
    private static GoalEndpoint goalEndpoint;

    // +----------------+
    // | DiarcComponent |
    // +----------------+

    /**
     * Called after the constructor. Sets additional parameters.
     * @param cmdLine the list of arguments and their values
     */
    @Override
    protected void parseArgs(CommandLine cmdLine) {

    }


    // +---------------------+
    // | WebSocketConfigurer |
    // + --------------------+

    /**
     * Add endpoints and allow the client to access them.
     * @param registry registry object that configures request mappings
     */
    @Override
    public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
        // registry.addHandler(...)

//        if I have a chat endpoint then
//            save chat endpoint into instance field
//            registry.addHandler(chatendpoit...)
    }
}
