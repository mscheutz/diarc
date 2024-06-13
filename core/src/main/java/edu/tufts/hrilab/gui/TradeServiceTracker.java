package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.*;
import jakarta.annotation.PostConstruct;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;

import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Tracks TRADE services, including initialization and periodic updates.
 * It registers services at startup and schedules periodic checks to manage service interactions.
 */
@Component
public class TradeServiceTracker {

  private static final Logger log = LoggerFactory.getLogger(TradeServiceTracker.class);

  /**
   * Initializes the component by registering all TRADE services and setting up a scheduled task
   * to track service calls.
   */
  @PostConstruct
  public void initializeAndRegister() {
    try {
      // Register only once upon initialization
      TRADE.registerAllServices(this, "");
    } catch (TRADEException e) {
      log.error("Error during initial TRADE registration", e);
    }

    // Create a ScheduledExecutorService to periodically update service tracking
    // need front end
    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
    executorService.scheduleAtFixedRate(this::trackAllTradeCalls, 5, 10, TimeUnit.SECONDS);
  }

  /**
   * Periodically tracks all TRADE service calls, invoking pre- and post-service hooks for monitoring
   * and logging purposes.
   */
  private void trackAllTradeCalls() {
    TRADEServiceInfo beforeService;
    TRADEServiceInfo afterService;
    try {
      beforeService = TRADE.getAvailableService(new TRADEServiceConstraints().name("beforeServiceWrapper"));
      afterService = TRADE.getAvailableService(new TRADEServiceConstraints().name("afterServiceWrapper"));
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }

    for (TRADEServiceInfo tsi: TRADE.getAvailableServices()) {
      if(tsi.equals(beforeService) || tsi.equals(afterService)) {
        continue;
      }
      try {
        TRADE.beforeService(beforeService, tsi);
        TRADE.afterService(afterService, tsi);
      } catch (TRADEException e) {
        throw new RuntimeException(e);
      }
    }
  }

  /**
   * Parses a set of service descriptions into a more structured form, detailing service names and their
   * respective argument types.
   *
   * @param serviceStrings A set of service descriptions.
   * @return A map of service names to their corresponding argument type arrays.
   */
  public static Map<String, String[]> parseServiceStrings(Set<String> serviceStrings) {
    Map<String, String[]> servicesToTrack = new HashMap<>();

    for (String serviceString : serviceStrings) {
      // Extract the service name (everything before the first "(")
      String serviceName = serviceString.substring(0, serviceString.indexOf("("));

      // Extract the argument types string (everything between "(" and ")")
      String argsString = serviceString.substring(serviceString.indexOf("(") + 1, serviceString.indexOf(")"));

      // Split the argument types string on commas not followed by a space, to avoid splitting generic types
      String[] serviceArgs = argsString.isEmpty() ? new String[]{} : argsString.split(",(?![^<>]*>)");

      // Convert each argument type to its full class name representation
      for (int i = 0; i < serviceArgs.length; i++) {
        serviceArgs[i] = convertToFullClassName(serviceArgs[i].trim());
      }

      servicesToTrack.put(serviceName, serviceArgs);
    }

    return servicesToTrack;
  }

  /**
   * Converts a simplified type string into a fully qualified class name, supporting both primitive
   * types and array representations.
   *
   * @param type The type string to convert.
   * @return The full class name of the given type.
   */
  private static String convertToFullClassName(String type) {
    Map<String, String> typeMappings = new HashMap<>();
    // Basic type mappings; extend this based on your needs
    typeMappings.put("int", "java.lang.Integer");
    typeMappings.put("long", "java.lang.Long");
    typeMappings.put("double", "java.lang.Double");
    typeMappings.put("float", "java.lang.Float");
    typeMappings.put("boolean", "java.lang.Boolean");
    typeMappings.put("char", "java.lang.Character");
    typeMappings.put("byte", "java.lang.Byte");
    typeMappings.put("short", "java.lang.Short");

    // Handle basic types and their array representations
    if (typeMappings.containsKey(type)) {
      return typeMappings.get(type);
    } else if (type.endsWith("[]") && typeMappings.containsKey(type.substring(0, type.length() - 2))) {
      return "[L" + typeMappings.get(type.substring(0, type.length() - 2)) + ";";
    }

    // Assume any other type is already a fully qualified class name
    return type;
  }

  /**
   * TRADE service method to be invoked before a service call for monitoring.
   *
   * @param args Arguments passed to the service call.
   */
  @TRADEService
  public void beforeServiceWrapper(Object[] args) {
    log.debug("Before service call... Args: {}", args);
  }

  /**
   * TRADE service method to be invoked after a service call for monitoring.
   *
   * @param args Arguments passed to the service call.
   */
  @TRADEService
  public void afterServiceWrapper(Object[] args) {
    log.debug("After service call... Args: {}", args);
  }
}
