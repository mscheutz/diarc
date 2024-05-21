/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import edu.tufts.hrilab.diarc.DiarcComponent;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.ConfigurableApplicationContext;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Bean;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.map.MapGui;


@Configuration
@EnableWebSocket
@ComponentScan(basePackages= "edu.tufts.hrilab")
public class DemoConfig implements WebSocketConfigurer {
  @Autowired
  private ConfigurableApplicationContext applicationContext;

  @Bean
  public MapComponent mapComponent() {
    MapComponent component = DiarcComponent.createInstance(MapComponent.class, "-map_folder /home/hrilab/code/diarc-old/maps/elevator_lab_test/ -start_floor 1");
    applicationContext.getBeanFactory().registerSingleton("mapComponent", component);
    return component;
  }

  @Bean
  public MapGui mapGui(MapComponent mapComponent) {
    return new MapGui(mapComponent);
  }

  public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
    registry.addHandler(mapGui(mapComponent()), "/map");
    registry.addHandler(new DemoComponent(), "/user");
  }
}