/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.listener.DatabaseListener;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.io.File;
import java.time.LocalDateTime;
import java.util.*;

/**
 * A WebSocket adapter for communication between the
 * <code>GoalManagerImpl</code> and the frontend Goal Manager GUI.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see edu.tufts.hrilab.action.GoalManagerComponent GoalManagerComponent (the
 * corresponding <code>GuiProvider</code> for this adapter)
 */
public class GoalManagerAdapter extends GuiAdapter implements DatabaseListener {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Path to the folder containing all <code>.asl</code> files. Depending on
   * method of launch, might require prefix <code>..</code> or might not.
   */
  public final String ACTION_SCRIPT_PATH =
          "../core/src/main/resources/config/edu/tufts/hrilab/action/asl";

  //==========================================================================
  // Fields
  //==========================================================================
  /**
   * TRADE service to submit an action.
   */
  private TRADEServiceInfo submitGoalService;

  /**
   * List of actions in the database.
   */
  private List<ActionDBEntry> actionList;

  /**
   * Utility object to write actions to an ASL file.
   */
  private final ActionScriptLanguageWriter aslWriter;

  //==========================================================================
  // Constructor
  //==========================================================================

  /**
   * Constructor.
   */
  public GoalManagerAdapter(Collection<String> groups) {
    super(groups);

    aslWriter = new ActionScriptLanguageWriter();
  }

  //==========================================================================
  // Methods
  //==========================================================================

  /**
   * Retrieve all actions from action database. Run during initialization.
   */
  @SuppressWarnings("unchecked assignment")
  private void retrieveActionList() {
    actionList = new ArrayList<>();

    try {
      actionList.addAll(
              TRADE.getAvailableService(
                      new TRADEServiceConstraints()
                              .returnType(Set.class).name("getAllActions").argTypes()
              ).call(Set.class)
      );
    } catch (TRADEException e) {
      log.error("Failed to retrieve actions from database", e);
    }

    actionList.sort(Comparator.comparing(e ->
            new ADBEWrapper(e).getActionSignature()));
  }

  /**
   * Find this instance's required TRADE services.
   */
  private void initializeServices() {
    try {
      submitGoalService = TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .name("submitGoal")
                      .argTypes(Predicate.class)
                      .returnType(long.class)
      );
    } catch (TRADEException e) {
      log.error("Failed to get TRADE services for goal manager");
    }
  }

  /**
   * Generates a filepath for the ASL writer to create a new file at.
   *
   * @return a String representation of a file path.
   */
  private String generateAslWriteFilePath() {
    LocalDateTime now = LocalDateTime.now();
    return ACTION_SCRIPT_PATH
            + "/custom/"
            + "gui-export-script-"
            + now
            .toString()
            // replace invalid or confusing chars
            .replace(':', '-')
            .replace('.', '-')
            + ".asl";
  }

  /**
   * Given a list of action IDs, export those actions to a file.
   *
   * @param toExport list of action IDs.
   */
  private void exportActionsAsFile(List<Number> toExport) {
    if (!new File(ACTION_SCRIPT_PATH + "/custom").mkdirs()) {
      log.error("Failed to ensure custom/ directory exists for asl files");
    }

    List<ActionDBEntry> toWrite = new ArrayList<>();
    for (Number id : toExport) {
      int i = id.intValue() - 1; // unshift index
      toWrite.add(actionList.get(i));
    }

    aslWriter.writeToFile(toWrite, generateAslWriteFilePath());
    notifyExportSuccessful();
  }

  /**
   * Send a message to the client that the ASL export was successful.
   */
  private void notifyExportSuccessful() {
    try {
      JsonObject json = new JsonObject();
      json.addProperty("export", "successful");
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("Failed to notify export successful", e);
    }
  }

  /**
   * Send a message to the client that the goal submission was successful.
   */
  private void notifySubmissionSuccessful() {
    try {
      JsonObject json = new JsonObject();
      json.addProperty("submit", "successful");
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("Failed to notify export successful", e);
    }
  }

  /**
   * Called on receipt of custom-action message.
   *
   * @param customAction the action string.
   */
  private void submitCustomAction(String customAction) {
    try {
      submitGoalService.call(
              void.class,
              Factory.createPredicate(customAction)
      );
    } catch (TRADEException e) {
      log.error("TRADEException occurred during invocation of custom action", e);
    }
  }

  /**
   * Called on receipt of form-action message.
   *
   * @param arguments the array of arguments.
   */
  private void submitFormAction(JsonArray arguments) {
    try {
      String actionName = arguments.get(0).getAsString();
      String[] args = new String[arguments.size()-1];
      for (int i = 1; i < arguments.size(); i++) {
        args[i-1] = arguments.get(i).getAsString();
      }
      submitGoalService.call(
              void.class,
              Factory.createPredicate(actionName, args)
      );
    } catch (TRADEException e) {
      log.error("TRADEException occurred during invocation of form action", e);
    }
  }

  /**
   * Submit a goal in agent-predicate form.
   *
   * @param agent a String representing the agent.
   * @param goal  a String representing the predicate.
   */
  private void submitGoal(String agent, String goal) {
    try {
      submitGoalService.call(
              void.class,
              Factory.createPredicate("goal", agent, goal)
      );
    } catch (TRADEException e) {
      log.error("TRADEException occurred during invocation of goal", e);
    }
  }

  /**
   * Starts a periodic update after the connection is established to the GUI
   * client.
   */
  private void onConnect() {
    // update cached actions
    retrieveActionList();

    JsonArray actions = new JsonArray();
    for (int i = 0; i < actionList.size(); i++) {
      JsonObject action = new JsonObject();
      action.addProperty("name", new ADBEWrapper(actionList.get(i)).getActionSignature());
      action.addProperty("id", i + 1); // id 0 is taken by the root
      actions.add(action);
    }
    JsonObject message = new JsonObject();
    message.add("actions", actions);
    message.addProperty("path", getPath());

    try {
      sendMessage(message);
    } catch (TRADEException e) {
      log.error("Couldn't send actions");
    }
  }

  //==========================================================================
  // Implementing methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  protected void init() {
    initializeServices();
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
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("type").getAsString()) {
      case "custom" -> {
        submitCustomAction(
          message.getAsJsonObject("formData").get("custom").getAsString()
        );
        notifySubmissionSuccessful();
      }
      case "form" -> {
        submitFormAction(message.getAsJsonArray("formData"));
        notifySubmissionSuccessful();
      }
      case "goal" -> {
        submitGoal(
          message.getAsJsonObject("formData").get("agent").getAsString(),
          message.getAsJsonObject("formData").get("goal").getAsString()
        );
        notifySubmissionSuccessful();
      }
      case "export" -> {
        List<Number> selected = new Gson().fromJson(message.getAsJsonArray("selected"), ArrayList.class);
        exportActionsAsFile(selected);
      }
      case "startup" -> onConnect();
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  public String getPathRoot() {
    return "goalManager";
  }

  //============================================================================
  // Implementing methods | DatabaseListener
  //============================================================================
  
  /**
   * Notifies the frontend of a new action to offer the user.
   * Called when an action is added to the database.
   * @param adb
   */
  @Override
  public void actionAdded(ActionDBEntry adb) {
    // COPIED CODE: creation of action list
    actionList = new ArrayList<>();

    try {
      actionList.addAll(
              TRADE.getAvailableService(
                      new TRADEServiceConstraints()
                              .returnType(Set.class).name("getAllActions").argTypes()
              ).call(Set.class)
      );
    } catch (TRADEException e) {
      log.error("Failed to retrieve actions from database", e);
    }

    actionList.sort(Comparator.comparing(e ->
            new ADBEWrapper(e).getActionSignature()));

    // COPIED CODE: onConnect()
    // update cached actions
    retrieveActionList();

    JsonArray actions = new JsonArray();
    for (int i = 0; i < actionList.size(); i++) {
      JsonObject action = new JsonObject();
      action.addProperty("name", new ADBEWrapper(actionList.get(i)).getActionSignature());
      action.addProperty("id", i + 1); // id 0 is taken by the root
      actions.add(action);
    }
    JsonObject message = new JsonObject();
    message.add("actions", actions);
    message.addProperty("path", getPath());

    try {
      sendMessage(message);
    } catch (TRADEException e) {
      log.error("Couldn't send actions");
    }

    throw new UnsupportedOperationException("Unimplemented method 'actionAdded'");
  }

  /**
   * Notifies the frontend to remove an action from display.
   * Called when an action is removed from the database.
   * @param adb
   */
  @Override
  public void actionRemoved(ActionDBEntry adb) {
    throw new UnsupportedOperationException("Unimplemented method 'actionRemoved'");
  }

}
