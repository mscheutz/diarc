/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import ai.thinkingrobots.trade.*;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.goal.GoalFilter;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.goal.schedule.ScheduleInfo;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.nio.file.Paths;
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
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalInfos").argTypes(GoalFilter.class));
      List<GoalInfo> goals = tsi.call(List.class, GoalFilter.ALL);

      JsonObject json = new JsonObject();
      json.add("goals", toJson(goals));
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("Error calling getGoalInfos service.", e);
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
    if (info.isLearningGoal) {
      //TODO: probably care about executeWhileLearning
      return;
    }
    JsonObject json = new JsonObject();
    List<GoalInfo> updatedGoals = new ArrayList<>();
    updatedGoals.add(info);
    json.add("goals", toJson(updatedGoals));
    json.addProperty("path", getPath());
    sendMessage(json);
  }

  private String convertGoalToReadableString(GoalInfo info) {
    Predicate goalPred = info.goal == null ? (Predicate) info.unboundGoal : info.goal;
    String predString = goalPred.toString();
    String[] splitRes = predString.split("[(),](?=[^\"][^\"(),]*[^\"][(),]|\".*\"(?![^,()]))");
    try {
      //Trim end parens from last arg - sloppy, do this better
      int resLen = splitRes.length;
      while (splitRes[resLen-1].endsWith(")")) {
        splitRes[resLen-1] = splitRes[resLen-1].substring(0,splitRes[resLen-1].length()-1);
      }

      String funcName = splitRes[0];
      switch (funcName) {
        case "followMe":
        case "followMeBlocking":
        case "follow":
          return "Follow";
        case "goToLocation":
          return String.format("Go to location '%s'", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "saveLocation":
          return String.format("Save location '%s'", splitRes[2]);
        case "deleteLocation":
          return String.format("Delete location '%s'", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "display":
          return String.format("Display message '%s'", splitRes[2]);
        case "sayText":
          return String.format("Say message '%s'", splitRes[2]);
        case "playVideo":
          return String.format("Play video '%s'", splitRes[2]);
        case "escort":
          if (splitRes.length == 6) {
            return String.format("Escort %s from %s to %s", splitRes[3], getLocationNameFromReference(Factory.createSymbol(splitRes[4])), getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
          } else {
            return String.format("Escort %s from current location to %s", splitRes[3], getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
          }
        case "escortToLocation":
          return String.format("Escort %s to %s", splitRes[2], getLocationNameFromReference(Factory.createSymbol(splitRes[3])));
        case "escortFromLocationToLocation":
          return String.format("Escort %s from %s to %s", splitRes[2], getLocationNameFromReference(Factory.createSymbol(splitRes[3])), getLocationNameFromReference(Factory.createSymbol(splitRes[4])));
        case "fetch":
          if (splitRes.length == 5) {
            return String.format("Fetch %s from %s", splitRes[2], getLocationNameFromReference(Factory.createSymbol(splitRes[3])));
          } else {
            return String.format("Fetch %s", splitRes[2]);
          }
        case "greet":
          return "Greet in " + splitRes[2];
        case "tour":
          return "Tour";
        case "clearScreen":
          return "Clear screen";
        case "displayFace":
          return "Display face";
        case "chargeTemi":
          return "Charging";
        case "goToThenSay":
          return String.format("Go to %s then say message", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "goToThenDisplay":
          return String.format("Go to %s then display message", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "goToThenPlay":
          return String.format("Go to %s then play video %s", getLocationNameFromReference(Factory.createSymbol(splitRes[2])), splitRes[3]);
        case "setVolume":
          return String.format("Set volume to %s", splitRes[2]);
        case "displayQRCode":
          return "Display Website QR";
        case "relocalize":
          return "Reposition";
        case "checkInKiosk":
          return String.format("Check in patients at location %s", getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "say":
          return String.format("Say " + splitRes[2]);
        case "startDemo":
          return "Start Demo";
        case "stopDemo":
          return "Stop Demo";
        case "reportBatteryLevel":
          return "Report battery level";
        case "sit":
          return "Sit";
        case "stand":
          return "Stand";
        case "goHome":
          return "Go to charging dock";
        case "learnNewTask":
          return "Learn a new action";
        case "chargeAtHome":
          return "Return to dock and charge";
        case "startSchedulingTask":
        case "scheduleGoal":
          return "Schedule a new task";
        case "modifyScheduledGoal":
          return "Modify a scheduled task";
        case "definePatrolPath":
          return "Learn a new patrol path";
        case "patrol":
          return String.format("Patrol along %s while performing %s", splitRes[2], splitRes[3]);
        case "playNextVideoAtLocation":
          return String.format("Show a video at location %s",getLocationNameFromReference(Factory.createSymbol(splitRes[2])));
        case "learnTourAction":
          return "Learning new tour sequence";
        case "reportInstructableActionsAction":
          return "Reporting list of instructable actions";
        case "reportTaughtActionsAction":
          return "Reporting list of taught actions";
        case "describeAllKnownLocationsActions":
          return "Reporting list of all saved locations";
        case "playNextVideo":
            return "Playing a video";
        case "playSpecificVideo":
          return String.format("Playing video \"%s\"",splitRes[2]);
        case "playSpecificVideoAtLocation":
          return String.format("Playing video \"%s\" at location \"%s\"", splitRes[2], getLocationNameFromReference(Factory.createSymbol(splitRes[3])));
        case "listSavedVideosAction":
          return "Listing all saved videos";
        case "listSavedImagesAction":
          return "Listing all saved Images";
        case "waitForDuration":
          return String.format("Waiting for %s seconds",splitRes[2]);
        default:
          return predString;
      }
    } catch (Exception e) {

    }
    return predString;
  }

  private String getLocationNameFromReference(Symbol ref) {
    String name = null;
    try {
      name = (String) TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapReferenceName").argTypes(Symbol.class)).call(String.class, ref);
    } catch (TRADEException e) {
      log.error("[getLocationNameFromReference] error getting location name from MapComponent for ref {}", ref, e);
    }
    if (name == null) {
      return ref.getName();
    } else {
      return name;
    }
  }

  /**
   * Retrieves a JsonArray of goal info objects.
   * Each object represents a goal using:
   *
   * <ul>
   *     <li><code>name: string</code></li>
   *     <li><code>actor: string</code></li>
   *     <li><code>currentAction: string</code></li>
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
    SimpleDateFormat simpleDateFormat = new SimpleDateFormat("HH:mm:ss");
    for (GoalInfo info : goals) {
      //AVA Specific Change - temp
      if (info.priorityTier == PriorityTier.SKIPPENDING && info.schedulingInfo == null) {
        continue;
      }
      JsonObject goalJson = new JsonObject();
      String goalString = convertGoalToReadableString(info);
      if (info.priorityTier == PriorityTier.KIOSK) {
        goalString = "(Kiosk) " + goalString;
      }
      goalJson.addProperty("name", goalString);
      goalJson.addProperty("actor", info.actor.getName());
//      goalJson.addProperty("currentAction", "");
      goalJson.addProperty("status", info.status.toString().toLowerCase());
      goalJson.addProperty("start", info.start == -1 ? "" : simpleDateFormat.format(new Date(info.start)));
      goalJson.addProperty("end", (info.end == -1) ? "" : simpleDateFormat.format(new Date(info.end)));
      goalJson.addProperty("priority", info.priority);
      goalJson.addProperty("id", info.gid);
      // a finished schedule still carries schedulingInfo in getPastGoals, so gate on a live (non-
      // terminated) status: a scheduled goal is ACTIVE while it exists in the scheduler and only
      // terminates on cancel/end. this also updates live -- the status change fires on schedule add
      if (info.schedulingInfo != null && !info.status.isTerminated()) {
        goalJson.addProperty("scheduled", true);
        ScheduleInfo scheduleInfo = info.schedulingInfo;
        SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss");
        goalJson.addProperty("start", scheduleInfo.startTime == null ? "Immediately" : timeFormat.format(scheduleInfo.startTime));
        goalJson.addProperty("end", scheduleInfo.endTime == null ? "Forever" : timeFormat.format(scheduleInfo.endTime));
        String intervalString = "N/A";
        if (scheduleInfo.period != -1) {
          if (scheduleInfo.period <= 60000) {
            intervalString = scheduleInfo.period / 1000 + " seconds";
          } else if (scheduleInfo.period <= 3600000) {
            intervalString = scheduleInfo.period / 60000 + " minutes";
          } else {
            intervalString = scheduleInfo.period / 3600000 + " hours";
          }
        }
        goalJson.addProperty("interval", intervalString);
      } else {
        goalJson.addProperty("scheduled", false);
      }
      if(info.failConditions != null) {
        goalJson.addProperty("failureReason", info.failConditions.getFailureReason().toString());
      }
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

  private void signalAvaExternalCommandReceived() {
    try {
      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
      service.call(void.class,"Received External Command...");
    } catch (TRADEException e) {
      log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
    }
    new Thread(() -> {
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        log.error("[handleMessage] exception sleeping in clearDisplay thread");
      }
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[handleMessage] Exception displaying text", e);
      }
    }).start();
  }

  /**
   * {@inheritDoc}
   *
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("method").getAsString()) {
      case "cancel":
        applyChangeToGoals(message.getAsJsonArray("ids"), cancelGoal);
        break;
      case "suspend":
        applyChangeToGoals(message.getAsJsonArray("ids"), suspendGoal);
        break;
      case "resume":
        applyChangeToGoals(message.getAsJsonArray("ids"),resumeGoal);
        break;
      case "startup":
        onConnect();
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
