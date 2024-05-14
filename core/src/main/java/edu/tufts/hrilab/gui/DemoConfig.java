/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;
//import edu.tufts.hrilab.movebase.map.MapGui;

@Configuration
@EnableWebSocket
@ComponentScan(basePackages= "edu.tufts.hrilab")
public class DemoConfig implements WebSocketConfigurer {

  public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
    registry.addHandler(new DemoComponent(), "/user");
//    registry.addHandler(new MapGui(), "/map");
  }

}