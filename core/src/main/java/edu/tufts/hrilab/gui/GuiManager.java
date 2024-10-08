/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADEException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;

/**
 * An implementation of the Spring server backend. Spins up a
 * <code>Handler</code> to handle the WebSocket connection and its messages,
 * and also provides some RESTful endpoints. For the frontend to function, this
 * program must be run in one of two ways:
 *
 * <ul>
 *     <li>Putting <code>SpringApplication.run(GuiManager.class, args);</code>
 *     in the <code>main()</code> method of your DIARC configuration file.
 *     Doing this will launch the <code>GuiManager</code> alongside your DIARC
 *     components.</li>
 *     <li>Running this class's <code>main()</code> method using
 *     <code>./gradlew launch -Pmain=edu.tufts.hrilab.gui.GuiManager</code>
 *     in the terminal. Note that this will cause a new TRADE container to be
 *     spawned, which requires that you configure your TRADE properties in a
 *     way that allows for mutual discovery of containers.</li>
 * </ul>
 *
 * @author Lucien Bao, Hengxu Li
 * @version 1.0
 * @see Handler Handler, which handles all WebSocket messages
 */
@SpringBootApplication
@EnableWebSocket
@ComponentScan(basePackages = "edu.tufts.hrilab")
public class GuiManager implements WebSocketConfigurer {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Static logger for this class.
   */
  private static final Logger log = LoggerFactory.getLogger(GuiManager.class);

  //==========================================================================
  // Constructor
  //==========================================================================
  /**
   * Constructor.
   */
  public GuiManager() {
  }

  //==========================================================================
  // Methods
  //==========================================================================
  /**
   * Helper method to convert the cross-origin URL string to a string array.
   *
   * @return a string array of allowed cross origins.
   */
  private String[] parseCorsOrigins() {
    return ConfigurationHolder.getCorsOrigins();
  }

  //==========================================================================
  // Implement methods | WebSocketConfigurer
  //==========================================================================

  /**
   * Add endpoints and allow the client to access them.
   *
   * @param registry registry object that configures request mappings
   */
  @Override
  public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
    try {
      registry.addHandler(Handler.createHandler(), "ws")
              .setAllowedOrigins(parseCorsOrigins());
    } catch(TRADEException e) {
      log.error("Could not create registry", e);
    }
  }

  static public void main(String[] args) {
    SpringApplication.run(GuiManager.class, args);
  }
}
