/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.event.ContextRefreshedEvent;
import org.springframework.context.event.EventListener;
import org.springframework.stereotype.Component;

import java.util.HashSet;
import java.util.Set;

@Component
@ConditionalOnFeature("enableTradeTracker")
public class TradeServiceTracker {

  private static final Logger log = LoggerFactory.getLogger(TradeServiceTracker.class);

  /**
   * Initializes the tracker by setting up after service hooks for all TRADE services.
   */
  @EventListener(ContextRefreshedEvent.class)
  public void onApplicationEvent(ContextRefreshedEvent event) {
    try {
      // will contain afterServiceWrapper([Ljava.lang.Object;) [MonitoringGroup]
      TRADE.registerAllServices(this, "MonitoringGroup");

      // Get wrappers for after service calls
      TRADEServiceInfo afterService = TRADE.getAvailableService(new TRADEServiceConstraints().name("afterServiceWrapper"));

      // Set up the exclusion set
      Set<String> excludedServices = new HashSet<>();
      excludedServices.add("afterServiceWrapper");
      excludedServices.add("sendMessage");
      excludedServices.add("getRules");
      excludedServices.add("getFacts");

      // Set up after hooks for all services
      for (TRADEServiceInfo service : TRADE.getAvailableServices()) {
        if (excludedServices.contains(service.serviceName)) {
          continue;
        }
        TRADE.afterService(afterService, service);
      }
    } catch (TRADEException e) {
      log.error("Error during initial TRADE setup", e);
    }
  }

  /**
   * A TRADE service method to be invoked before a service call for monitoring.
   *
   * @param args Arguments passed to the service call.
   */
  @TRADEService
  public void beforeServiceWrapper(Object[] args) {
    log.debug("Before service call... Args: {}", args);
  }

  /**
   * A TRADE service method to be invoked after a service call for monitoring.
   *
   * @param args Arguments passed to the service call.
   */
  @TRADEService
  public void afterServiceWrapper(Object[] args) {
    log.info("After service call... Args: {}", args);
  }
}
