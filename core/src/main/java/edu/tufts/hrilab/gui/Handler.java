/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.*;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * A WebSocket handler which instantiates <code>GuiAdapter</code>s when their
 * respective <code>GuiProvider</code> are registered with TRADE,
 * distributes incoming messages to the relevant <code>GuiAdapter</code>s,
 * and collects outgoing messages for transmission to the frontend.
 *
 * @author Lucien Bao
 * @version 1.0
 */
public class Handler extends TextWebSocketHandler {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Static logger instance.
   */
  private static final Logger log = LoggerFactory.getLogger(Handler.class);

  //==========================================================================
  // Fields
  //==========================================================================
  /**
   * The current session.
   */
  private WebSocketSession session;

  /**
   * Maps adapter paths to their references.
   */
  private final Map<String, GuiAdapter> adapterMap;

  /**
   * Lock instance for adapter map.
   */
  private final Object adapterLock = new Object();

  //==========================================================================
  // Constructors
  //==========================================================================

  /**
   * Constructor. Private so as to force use of the factory method.
   */
  private Handler() {
    session = null;
    adapterMap = new HashMap<>();
  }

  //==========================================================================
  // Methods
  //==========================================================================

  /**
   * Factory method. Handles construction, registration with TRADE, and
   * registration to add new <code>GuiAdapter</code>s when their corresponding
   * <code>GuiProvider</code>s join.
   *
   * @return the handler instance.
   * @throws TRADEException if registration of the instance's services fails,
   *                        if registration of any <code>GuiAdapter</code> fails, or if registration
   *                        for join notifications fails.
   */
  public static Handler createHandler() throws TRADEException {
    Handler instance = new Handler();

    TRADE.registerAllServices(instance, (String) null);

    for (TRADEServiceInfo tsi : getAdapterClassNameServices())
      instance.registerAdapter(tsi);
    TRADE.requestNotification(
            instance,
            "joined",
            new TRADEServiceConstraints().name("getAdapterClassNames"),
            null,
            "registerAdapter"
    );

    return instance;
  }

  /**
   * TRADE service to register new adapters to this handler.
   *
   * @param tsi a TRADE service that returns the new adapter's class name.
   *            Provided by the notifier.
   * @throws TRADEException if registration of a new adapter with TRADE fails.
   */
  @TRADEService
  public void registerAdapter(TRADEServiceInfo tsi) throws TRADEException {
    String[] names = tsi.call(String[].class);
    Collection<String> groups = tsi.getGroups();
    for (String name : names) {
      try {
        @SuppressWarnings("unchecked")
        Class<? extends GuiAdapter> clazz =
                (Class<? extends GuiAdapter>) Class.forName(name);
        GuiAdapter handler = GuiAdapter.createAndRegisterInstance(clazz, groups);
        synchronized (adapterLock) {
          adapterMap.put(handler.getPath(), handler);
        }
      } catch (ClassNotFoundException e) {
        log.error("Could not find class for name {}", name, e);
      } catch (NoSuchMethodException | InvocationTargetException |
               InstantiationException | IllegalAccessException e) {
        log.error("Could not instantiate {}", name, e);
      }
    }
    sendAdapterUpdate();
  }

  /**
   * TRADE service to send messages to the client.
   *
   * @param message the message to send.
   */
  @TRADEService
  public synchronized void sendMessage(String message) throws IOException {
    if (session == null) return;
    session.sendMessage(new TextMessage(message));
  }

  /**
   * Send the client an update on which adapters are available.
   */
  private void sendAdapterUpdate() {
    JsonArray paths = new JsonArray();
    synchronized (adapterLock) {
      for (String path : adapterMap.keySet()) {
        paths.add(path);
      }
    }

    JsonObject object = new JsonObject();
    object.add("paths", paths);
    try {
      sendMessage(object.toString());
    } catch (IOException e) {
      log.error("Could not send adapter update", e);
    }
  }

  /**
   * Get all the TRADE services which return <code>GuiAdapter</code> class
   * names.
   *
   * @return a collection of all such TRADE services
   */
  private static Collection<TRADEServiceInfo> getAdapterClassNameServices() {
    return TRADE.getAvailableServices(new TRADEServiceConstraints()
            .returnType(String[].class)
            .name("getAdapterClassNames")
            .argTypes()
    );
  }

  //==========================================================================
  // Implement methods | TextWebSocketHandler
  //==========================================================================

  /**
   * Callback to send a setup message to the client on connection setup.
   *
   * @param session the WebSocket connection just established.
   * @throws Exception ignored.
   */
  @Override
  public void afterConnectionEstablished(@Nonnull WebSocketSession session)
          throws Exception {
    super.afterConnectionEstablished(session);
    this.session = session;
    sendAdapterUpdate();
  }

  /**
   * Callback to handle messages sent from the client.
   *
   * @param session the WebSocket connection the message uses.
   * @param message the message.
   * @throws Exception ignored.
   */
  @Override
  protected void handleTextMessage(@Nonnull WebSocketSession session,
                                   @Nonnull TextMessage message)
          throws Exception {
    super.handleTextMessage(session, message);
    JsonElement jelem = new Gson().fromJson(message.getPayload(), JsonElement.class);
    JsonObject payload = jelem.getAsJsonObject();
    GuiAdapter adapter;

    synchronized (adapterLock) {
      adapter = adapterMap.get(payload.get("path").getAsString());
    }

    if (adapter != null) {
      adapter.handleMessage(payload);
    }
  }

  /**
   * Callback to reset the session on close.
   *
   * @param session the closing WebSocket connection.
   * @param status  ignored.
   * @throws Exception ignored.
   */
  @Override
  public void afterConnectionClosed(@Nonnull WebSocketSession session,
                                    @Nonnull CloseStatus status)
          throws Exception {
    super.afterConnectionClosed(session, status);
    this.session = null;
  }
}
