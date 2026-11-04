/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.util.IdGenerator;

import java.util.*;

/**
 * TradeServiceAdapter. Adapter class that provides information to the frontend
 * about what TRADE services are available. Also serves as an endpoint for
 * the frontend to invoke TRADE services.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see TradeServiceComponent
 */
public class TradeServiceAdapter extends GuiAdapter {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Unified representation (key) for 'no groups' in the service map.
   */
  @SuppressWarnings("FieldCanBeLocal")
  private final String NO_GROUPS = "groupless";

  //==========================================================================
  // Fields
  //==========================================================================
  /**
   * Map that groups services by their TRADE groups.
   */
  final Map<String, List<Service>> groupMap;

  /**
   * Map that identifies services by their IDs.
   */
  final Map<Long, Service> idMap;

  /**
   * ID generator for services.
   */
  final IdGenerator idGenerator;

  //==========================================================================
  // Constructors
  //==========================================================================

  /**
   * Constructor.
   *
   * @param groups the groups that the associated DIARC component belongs to
   */
  public TradeServiceAdapter(Collection<String> groups) {
    super(groups);
    groupMap = new HashMap<>();
    idMap = new HashMap<>();
    idGenerator = new IdGenerator();
  }

  //==========================================================================
  // Static methods
  //==========================================================================

  /**
   * Convert a TRADE service into a JSON representation.
   *
   * @param tsi the TRADE service.
   * @param id  the service's identifier.
   * @return a JsonObject.
   */
  private static JsonObject jsonify(TRADEServiceInfo tsi, long id) {
    JsonArray params = new JsonArray();
    JsonArray paramTypes = new JsonArray();
    for (int i = 0; i < tsi.serviceParameterNames.length; i++) {
      params.add(tsi.serviceParameterNames[i]);
      paramTypes.add(tsi.serviceParameterTypeNames[i]);
    }
    JsonObject json = new JsonObject();
    json.addProperty("name", tsi.serviceName);
    json.addProperty("returnType", tsi.serviceReturnTypeName);
    json.add("params", params);
    json.add("paramTypes", paramTypes);
    json.addProperty("id", id);
    return json;
  }

  /**
   * Replace URL-unsafe characters from a given string.
   *
   * @param str the string to clean.
   * @return a URL-safe version.
   */
  private static String toUrlSafe(String str) {
    return str.replace(":", "-")
            .replace(" ", "_");
  }

  //==========================================================================
  // Instance methods
  //==========================================================================

  /**
   * Convert a String representation of an object to an object of the
   * appropriate type.
   *
   * @param raw   the raw String representation.
   * @param clazz the class to convert to.
   * @param <T>   the type desired.
   * @return an object of the given class.
   */
  @SuppressWarnings("unchecked")
  private <T> T convert(String raw, Class<T> clazz) {
    return (T) switch (clazz.getName()) {
      case "java.lang.Integer":
        yield Integer.valueOf(raw);
      case "java.lang.Double":
        yield Double.valueOf(raw);
      case "java.lang.Float":
        yield Float.valueOf(raw);
      case "java.lang.Long":
        yield Long.valueOf(raw);
      case "java.lang.Short":
        yield Short.valueOf(raw);
      case "java.lang.Byte":
        yield Byte.valueOf(raw);
      case "java.lang.Boolean":
        yield Boolean.valueOf(raw);
      case "java.lang.Character":
        yield raw.charAt(0);
      case "java.lang.String":
        yield raw;
      case "edu.tufts.hrilab.fol.Symbol":
        yield Factory.createSymbol(raw);
      case "edu.tufts.hrilab.fol.Predicate":
      case "edu.tufts.hrilab.fol.Term":
        yield Factory.createPredicate(raw);
      case "edu.tufts.hrilab.belief.common.MemoryLevel":
        if (raw.equalsIgnoreCase("universal"))
          yield MemoryLevel.UNIVERSAL;
        else if (raw.equalsIgnoreCase("episodic"))
          yield MemoryLevel.EPISODIC;
        else if (raw.equalsIgnoreCase("working"))
          yield MemoryLevel.WORKING;
        else
          throw new IllegalArgumentException("Invalid memory level");
      default:
        throw new UnsupportedOperationException("Conversion for {} unavailable");
    };
  }

  /**
   * Update the client on available trade services.
   */
  private void update() {
    JsonObject message = new JsonObject();
    message.addProperty("path", getPath());

    // Parallel arrays
    JsonArray groups = new JsonArray();
    JsonArray services = new JsonArray();
    for (Map.Entry<String, List<Service>> entry : groupMap.entrySet()) {
      JsonArray groupServices = new JsonArray();
      for (Service service : entry.getValue()) {
        groupServices.add(service.json());
      }
      groups.add(entry.getKey());
      services.add(groupServices);
    }
    message.add("groups", groups);
    message.add("services", services);
    try {
      sendMessage(message);
    } catch (TRADEException e) {
      log.error("Failed to send message");
    }
  }

  /**
   * Invoke the requested TRADE service.
   *
   * @param message the WebSocket message from the client indicating
   *                the TRADE service, its args, and its groups.
   * @throws TRADEException if the invocation fails.
   */
  private void invokeServiceAndRespond(JsonObject message) throws TRADEException {
    TRADEServiceInfo tsi = idMap.get(message.get("id").getAsLong()).tsi;
    // List of String, actually
    List<Object> rawArgs = new Gson().fromJson(message.getAsJsonArray("args"), ArrayList.class);
    Object[] args = new Object[rawArgs.size()];
    JsonObject response = new JsonObject();
    response.addProperty("path", getPath());
    try {
      for (int i = 0; i < args.length; i++) {
        args[i] = convert(
                (String) rawArgs.get(i),
                tsi.getServiceParameterTypes()[i]
        );
      }
      if (tsi.getServiceReturnType().equals(void.class)) {
        response.addProperty("result", "No result from void method.");
      } else {
        response.addProperty("result", tsi.call(tsi.getServiceReturnType(), args).toString());
      }
    } catch (IllegalArgumentException e) {
      response.addProperty("result", "An illegal argument was provided.");
    } catch (UnsupportedOperationException e) {
      response.addProperty("result", "An argument was provided for which there is"
              + " currently no conversion.");
    } catch (TRADEException e) {
      response.addProperty("result", "TRADE call failed.");
    }
    sendMessage(response);
  }

  //==========================================================================
  // Implement methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  protected void init() {
    Collection<TRADEServiceInfo> availableServices =
            TRADE.getAvailableServices();
    for (TRADEServiceInfo tsi : availableServices) {
      Collection<String> groups = tsi.getGroups();
      // No groups
      if (groups == null || groups.isEmpty()) {
        List<Service> groupServices =
                groupMap.getOrDefault(NO_GROUPS, new ArrayList<>());
        long id = idGenerator.getNext();
        Service service = new Service(tsi, id);
        groupServices.add(service);
        groupMap.put(NO_GROUPS, groupServices);
        idMap.put(id, service);
      }
      // Has groups
      else {
        for (String group : groups) {
          if (group == null) continue;
          group = toUrlSafe(group);

          List<Service> groupServices =
                  groupMap.getOrDefault(group, new ArrayList<>());
          long id = idGenerator.getNext();
          Service service = new Service(tsi, id);
          groupServices.add(service);
          groupMap.put(group, groupServices);
          idMap.put(id, service);
        }
      }
    }

    for (List<Service> list : groupMap.values()) {
      list.sort(Service.COMPARATOR);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    if (message.has("setup"))
      update();
    else
      try {
        invokeServiceAndRespond(message);
      } catch (TRADEException e) {
        log.error("Failed to send response");
      }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected String getPathRoot() {
    return "tradeService";
  }

  //==========================================================================
  // Inner class | Service
  //==========================================================================

  /**
   * A record class for storing a TRADE service and its JSON representation.
   *
   * @param tsi  a TRADE service.
   * @param id   a unique identifier for the service.
   * @param json a JSON representation of the service.
   */
  private record Service(TRADEServiceInfo tsi, long id, JsonObject json) {
    //======================================================================
    // Static fields
    //======================================================================
    /**
     * Comparator used for sorting services.
     */
    static final Comparator<Service> COMPARATOR =
            Comparator.comparing(s -> s.tsi.serviceString);

    //======================================================================
    // Non-canonical constructors
    //======================================================================

    /**
     * Convenience constructor that automatically generates the JSON
     * depending on the input TRADE service.
     *
     * @param tsi the TRADE service.
     * @param id  the service's identifier.
     */
    Service(TRADEServiceInfo tsi, long id) {
      this(tsi, id, jsonify(tsi, id));
    }
  }
}
