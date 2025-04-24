/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.simspeech.gui;

import ai.thinkingrobots.trade.*;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.GuiAdapter;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;

/**
 * A WebSocket adapter for communication between DIARC and the frontend
 * Chat Viewer GUI.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see edu.tufts.hrilab.slug.listen.ListenerComponent ListenerComponent
 * (the corresponding <code>GuiProvider</code> for this adapter)
 * @see SimSpeechRecognitionComponentVis
 */
public class ChatAdapter extends GuiAdapter {
  //==========================================================================
  // Fields
  //==========================================================================

  private TRADEServiceInfo diarcAgentsService;

  private TRADEServiceInfo dialogueHistoryService;


  //==========================================================================
  // Constructor
  //==========================================================================

  /**
   * Constructor.
   */
  public ChatAdapter(Collection<String> groups) {
    super(groups);
  }

  //==========================================================================
  // Methods
  //==========================================================================

  /**
   * Get the current DIARC agents to update the frontend with the list of
   * robot names. In case the TRADE service for this is not yet available,
   * also register to get notified when it becomes so.
   */
  private void registerForDiarcAgents() {
    // Register to get notified of DIARC agents. To protect against a race
    // condition, this must happen before trying to get them now.
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
              new TRADEServiceConstraints().returnType(Set.class).name("getDiarcAgents").argTypes(),
              "populateDiarcAgents"
      );
    } catch (TRADEException e) {
      log.error("Failed to request notification for DIARC agents", e);
    }
  }

  /**
   * Callback to send names to the frontend when the DIARC agents join.
   * This is a helper method in case we need to register for notifications.
   *
   * @param service the <code>getDiarcAgents()</code> TRADE service, provided
   *                by the notifier.
   */
  @TRADEService
  public void populateDiarcAgents(TRADEServiceInfo service) {
    diarcAgentsService = service;

    try {
      @SuppressWarnings("unchecked")
      Set<Symbol> agents = diarcAgentsService.call(Set.class);
      Set<String> diarcAgents = new HashSet<>();
      agents.forEach(agent -> diarcAgents.add(agent.getName()));
      sendDiarcAgents(diarcAgents);
      log.debug("DIARC agents received");
    } catch (TRADEException e) {
      log.error("Error trying to get diarc agents.", e);
    }
  }

  private void sendDiarcAgents(Set<String> diarcAgents) {
    try {
      JsonObject response = new JsonObject();
      response.addProperty("path", getPath());
      response.addProperty("names", Arrays.toString(diarcAgents.toArray()));
      sendMessage(response);
    } catch (TRADEException e) {
      log.error("Failed to send names to chat", e);
    }
  }

  /**
   * Registers this component's <code>sendMessage()</code> TRADE service
   * and links it to the <code>DialogueComponent</code>'s notification
   * registry so that it will be called when new <code>Utterances</code> are
   * added. In case the TRADE service for the registration is not yet
   * available, also register to get notified when it becomes so.
   */
  private void registerForDialogueService() {
    // Register to get notified of registration for dialogue history
    // notifications. To protect against a race condition, this must happen
    // before trying to get them now.
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
              new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications").argTypes(TRADEServiceInfo.class),
              "registerDialogue"
      );
    } catch (TRADEException e) {
      log.error("Failed to register for registration of dialogue history notifications", e);
    }
  }

  /**
   * Callback to register for dialogue history notifications when Dialogue
   * is up. This is a helper method in case we need to register for
   * notifications for the Dialogue notifications.
   *
   * @param service requested service
   * @throws TRADEException if the TRADE service call fails.
   */
  @TRADEService
  public void registerDialogue(TRADEServiceInfo service) throws TRADEException {
    // get existing dialogue history
    populateDialogueHistory(TRADE.getAvailableService(new TRADEServiceConstraints().name("getDialogueHistory").argTypes()));

    // register to be notified of dialogue changes
    TRADEServiceInfo sendUtteranceService = TRADE.getAvailableService(
            new TRADEServiceConstraints().name("sendUtterance").argTypes(Utterance.class)
    );
    service.call(void.class, sendUtteranceService);
    log.debug("Dialogue registered");
  }

  /**
   * Sends a message to the chat window.
   *
   * @param utterance what the robot is saying.
   */
  @TRADEService
  public void sendUtterance(Utterance utterance) {
    // send to client
    try {
      JsonObject json = new JsonObject();
      json.addProperty("message", utterance.getWordsAsString());
      json.addProperty("sender", utterance.getSpeaker().toString());
      json.addProperty("recipient", utterance.getAddressee().toString());
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("Failed to send message to chat");
    }
  }

  private void populateDialogueHistory(TRADEServiceInfo service) {
    dialogueHistoryService = service;
    try {
      List<Utterance> dialogueHistory = dialogueHistoryService.call(List.class);
      sendDialogueHistory(dialogueHistory);
    } catch (TRADEException e) {
      log.error("Failed to get dialogue history");
    }
  }

  private void sendDialogueHistory(List<Utterance> dialogueHistory) {
    dialogueHistory.forEach(utterance -> sendUtterance(utterance));
  }

  //==========================================================================
  // Implement methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  protected void init() {
    registerForDiarcAgents();
    registerForDialogueService();
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected boolean providesTradeServices() {
    return true;
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  public String getPathRoot() {
    return "chat";
  }

  private void onConnect() {
    // send diarc agents
    if (diarcAgentsService != null) {
      populateDiarcAgents(diarcAgentsService);
    }

    // send existing dialogue history
    if (dialogueHistoryService != null) {
      populateDialogueHistory(dialogueHistoryService);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    if (message.has("method") && message.get("method").getAsString().equals("startup")) {
      onConnect();
      return;
    }

    String data = message.get("message").getAsString();
    Symbol sender = Factory.createSymbol(message.get("sender").getAsString());
    Symbol recipient = Factory.createSymbol(message.get("recipient").getAsString());

    Utterance utt = new Utterance.Builder()
            .setWords(Arrays.asList(data.split(" ")))
            .setSpeaker(sender)
            .setAddressee(recipient)
            .setIsInputUtterance(true)
            .build();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("reportRecognizedSpeech").
              argTypes(Utterance.class)
      ).call(void.class, utt);
    } catch (TRADEException e) {
      log.error("Failed to report utterance", e);
    }
  }
}
