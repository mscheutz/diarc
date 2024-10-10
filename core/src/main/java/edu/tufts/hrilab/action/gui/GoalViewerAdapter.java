/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import ai.thinkingrobots.trade.*;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.text.SimpleDateFormat;
import java.util.*;

/**
 * A WebSocket adapter for communication between the
 * <code>GoalManagerImpl</code> and the frontend Goal Viewer GUI.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see edu.tufts.hrilab.action.GoalManagerComponent GoalManagerComponent (the
 * <code>GuiProvider</code> for this adapter)
 */
public class GoalViewerAdapter extends GuiAdapter {
  //==========================================================================
  // Fields
  //==========================================================================

  /**
   * TRADE service to cancel a goal.
   */
  private TRADEServiceInfo cancelGoal;

  /**
   * TRADE service to suspend a goal.
   */
  private TRADEServiceInfo suspendGoal;

  /**
   * TRADE service to resume a goal.
   */
  private TRADEServiceInfo resumeGoal;

//  private Map<Long, GoalInfo> allGoals = new HashMap<>();

  //==========================================================================
  // Constructor
  //==========================================================================

  /**
   * Constructor.
   */
  public GoalViewerAdapter(Collection<String> groups) {
    super(groups);
  }

  private void updateGoals() {
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllGoals").argTypes());
      List<GoalInfo> goals = tsi.call(List.class);

      JsonObject json = new JsonObject();
      json.add("goals", toJson(goals));
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("Error calling getAllGoals service.", e);
    }
  }

  private void registerForNotifications() {
    try {
      TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
      TRADEServiceInfo notificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleGoalNotification").argTypes(GoalInfo.class));
      registerService.call(void.class, notificationHandler, NotificationType.GOAL_STATUS_CHANGE);
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }
  }

  @TRADEService
  public void handleGoalNotification(GoalInfo info) throws TRADEException {
    JsonObject json = new JsonObject();
    List<GoalInfo> updatedGoals = new ArrayList<>();
    updatedGoals.add(info);
    json.add("goals", toJson(updatedGoals));
    json.addProperty("path", getPath());
    sendMessage(json);
  }

  /**
   * Retrieves a JsonArray of goal info objects.
   * Each object represents a goal using:
   *
   * <ul>
   *     <li><code>name: string</code></li>
   *     <li><code>actor: string</code></li>
   *     <li><code>status: string</code></li>
   *     <li><code>start: string</code></li>
   *     <li><code>end: string</code></li>
   *     <li><code>priority: number</code></li>
   *     <li><code>id: number</code></li>
   * </ul>
   * <p>
   * where <code>string</code> translates to a Java <code>String</code>
   * and <code>number</code> translates to a Java <code>long</code> or
   * <code>int</code>.
   *
   * @param goals a list of goals to convert.
   * @return a JSON array.
   */
  private JsonArray toJson(List<GoalInfo> goals) {
    JsonArray result = new JsonArray();
    SimpleDateFormat simpleDateFormat = new SimpleDateFormat("hh:mm:ss");
    for (GoalInfo info : goals) {
      JsonObject goalJson = new JsonObject();
      goalJson.addProperty("name", info.unboundGoal.toString());
      goalJson.addProperty("actor", info.actor.getName());
      goalJson.addProperty("status", info.status.toString().toLowerCase());
      goalJson.addProperty("start", simpleDateFormat.format(new Date(info.start)));
      goalJson.addProperty("end", (info.end == -1) ? "" : simpleDateFormat.format(new Date(info.end)));
      goalJson.addProperty("priority", info.priority);
      goalJson.addProperty("id", info.gid);
      result.add(goalJson);
    }
    return result;
  }

  /**
   * Cancel, resume, or suspend a group of goals specified in a
   * <code>JSONArray</code> of IDs.
   *
   * @param goals JSON array of goal IDs.
   * @param tsi   the change to apply.
   */
  private void applyChangeToGoals(JsonArray goals, TRADEServiceInfo tsi) {
    for (JsonElement id : goals) {
      try {
        tsi.call(void.class, id.getAsLong());
      } catch (TRADEException e) {
        log.error("Failed to apply change to goal", e);
      }
    }
  }

  /**
   * Sends initial goal information on connection to client.
   */
  private void onConnect() {
    updateGoals();
  }

  //==========================================================================
  // Implementing methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  protected void init() {
    try {
      cancelGoal = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("cancelGoal").argTypes(Long.class));
      suspendGoal = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("suspendGoal").argTypes(Long.class));
      resumeGoal = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("resumeGoal").argTypes(Long.class));

      updateGoals();

      registerForNotifications();
    } catch (TRADEException e) {
      log.error("Failed to find TRADE services", e);
      throw new RuntimeException(e);
    }
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
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("method").getAsString()) {
      case "cancel" -> applyChangeToGoals(message.getAsJsonArray("ids"),
              cancelGoal);
      case "suspend" -> applyChangeToGoals(message.getAsJsonArray("ids"),
              suspendGoal);
      case "resume" -> applyChangeToGoals(message.getAsJsonArray("ids"),
              resumeGoal);
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
    return "goalViewer";
  }

}
