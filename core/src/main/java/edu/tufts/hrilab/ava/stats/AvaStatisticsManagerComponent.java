/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.ava.stats;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.diarc.DiarcComponent;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.concurrent.locks.ReentrantLock;

public class AvaStatisticsManagerComponent extends DiarcComponent {

  private Path statisticsDir;
  private Path statisticsFile;
  private BufferedWriter writer;
  private final ReentrantLock writeLock = new ReentrantLock();
  private boolean headerWritten = false;

  @Override
  protected void init() {
    String home = System.getProperty("user.home");
    if (home == null || home.isEmpty()) {
      home = System.getProperty("user.dir", System.getProperty("java.io.tmpdir", "."));
    }
    statisticsDir = Paths.get(home, ".diarc", "statistics");
    try {
      Files.createDirectories(statisticsDir);
    } catch (IOException e) {
      log.error("[init] Could not create statistics directory: {}", statisticsDir, e);
      return;
    }
    statisticsFile = statisticsDir.resolve("ava_statistics.csv");
    try {
      writer = Files.newBufferedWriter(statisticsFile, StandardCharsets.UTF_8,
          StandardOpenOption.CREATE, StandardOpenOption.APPEND);
      // If file already has content (e.g. from a previous DIARC run), do not write header again.
      if (Files.size(statisticsFile) > 0) {
        headerWritten = true;
      }
    } catch (IOException e) {
      log.error("[init] Could not open statistics file: {}", statisticsFile, e);
    }
  }

  @Override
  protected void postRegistrationInit() {
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
          new TRADEServiceConstraints().name("registerForAvaStatisticsNotifications").argTypes(TRADEServiceInfo.class),
          "registerAvaStatisticsCallback");
    } catch (TRADEException e) {
      log.error("[postRegistrationInit] Error registering for Ava statistics notifications.", e);
    }
    // Notify when GoalManager is available (same pattern as registerForDialogueHistoryNotifications).
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
          new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class),
          "onGoalManagerRegistrationAvailable");
    } catch (TRADEException e) {
      log.error("[postRegistrationInit] Error registering for goal manager availability.", e);
    }
  }

  /**
   * Called by TRADE when registerGoalManagerNotification becomes available (now or when joined).
   * Registers our callback for GOAL_STATUS_CHANGE notifications.
   */
  @TRADEService
  public void onGoalManagerRegistrationAvailable(TRADEServiceInfo registerService) throws TRADEException {
    TRADEServiceInfo myHandler = getMyService("avaStatisticsGoalNotification", GoalInfo.class);
    registerService.call(void.class, myHandler, NotificationType.GOAL_STATUS_CHANGE);
    log.info("[onGoalManagerRegistrationAvailable] Registered for goal status notifications.");
  }

  /**
   * Called by GoalManager on every goal status change. Logs one row per transition
   * (supports cumulative active time). Skips learning goals.
   * <p>
   * Only top-level goals are reported: the notifier is only added to Goals when they
   * are submitted via {@link edu.tufts.hrilab.action.GoalManagerComponent#submitGoal}
   * or {@link edu.tufts.hrilab.action.manager.ScheduledGoalManager}; subgoals created
   * in {@link edu.tufts.hrilab.action.execution.Context#createChild} never get the
   * notifier and never call setStatus to external listeners.
   * </p>
   * <p>
   * Goal name uses the predicate name (same identity as in GoalViewerAdapter / GUI):
   * goal.getName() or unboundGoal.getName().
   * </p>
   */
  /** TRADE service name is unique so GoalViewerAdapter.getAvailableService("handleGoalNotification") stays unique. */
  @TRADEService
  public void avaStatisticsGoalNotification(GoalInfo info) throws TRADEException {
    if (info.isLearningGoal) {
      return;
    }
    String goalName = info.goal != null ? info.goal.getName()
        : (info.unboundGoal != null ? info.unboundGoal.getName() : "unknown");
    boolean scheduled = info.schedulingInfo != null;
    boolean urgentGoHome = AvaStatisticsEvent.URGENT_GO_HOME_GOAL_NAME.equals(goalName);
    AvaStatisticsEvent event = AvaStatisticsEvent.builder(AvaStatisticsEvent.Kind.GOAL_STATUS)
        .timestampMs(System.currentTimeMillis())
        .goalGid(info.gid)
        .goalName(goalName)
        .goalStatus(info.status.name())
        .goalStartMs(info.start >= 0 ? Long.valueOf(info.start) : null)
        .goalEndMs(info.end >= 0 ? info.end : null)
        .scheduled(scheduled)
        .urgentGoHome(urgentGoHome)
        .build();
    onAvaStatisticsEvent(event);
  }

  @TRADEService
  public void registerAvaStatisticsCallback(TRADEServiceInfo service) {
    TRADEServiceInfo myService = getMyService("onAvaStatisticsEvent", AvaStatisticsEvent.class);
    try {
      service.call(void.class, myService);
    } catch (TRADEException e) {
      log.error("[registerAvaStatisticsCallback] Error registering callback with Ava statistics producer.", e);
    }
  }

  @TRADEService
  public void onAvaStatisticsEvent(AvaStatisticsEvent event) {
    if (writer == null) {
      return;
    }
    writeLock.lock();
    try {
      if (!headerWritten) {
        writer.write("timestamp_ms,kind,dock_name,cumulative_distance_m,segment_start_timestamp_ms,drive_status_end,goal_gid,goal_name,goal_status,goal_start_ms,goal_end_ms,scheduled,urgent_go_home");
        writer.newLine();
        writer.flush();
        headerWritten = true;
      }
      StringBuilder row = new StringBuilder();
      row.append(event.getTimestampMs()).append(',');
      row.append(event.getKind().name()).append(',');
      row.append(escapeCsv(event.getDockName())).append(',');
      row.append(event.getCumulativeDistanceM() != null ? event.getCumulativeDistanceM() : "").append(',');
      row.append(event.getSegmentStartTimestampMs() != null ? event.getSegmentStartTimestampMs() : "").append(',');
      row.append(escapeCsv(event.getDriveStatusEnd()));
      if (event.getKind() == AvaStatisticsEvent.Kind.GOAL_STATUS) {
        row.append(',').append(event.getGoalGid() != null ? event.getGoalGid() : "");
        row.append(',').append(escapeCsv(event.getGoalName()));
        row.append(',').append(escapeCsv(event.getGoalStatus()));
        row.append(',').append(event.getGoalStartMs() != null ? event.getGoalStartMs() : "");
        row.append(',').append(event.getGoalEndMs() != null ? event.getGoalEndMs() : "");
        row.append(',').append(event.getScheduled() != null && event.getScheduled() ? "true" : "false");
        row.append(',').append(event.getUrgentGoHome() != null && event.getUrgentGoHome() ? "true" : "false");
      } else {
        row.append(",,,,,,");
      }
      writer.write(row.toString());
      writer.newLine();
      writer.flush();
    } catch (IOException e) {
      log.error("[onAvaStatisticsEvent] Error writing statistics event.", e);
    } finally {
      writeLock.unlock();
    }
  }

  private static String escapeCsv(String value) {
    if (value == null) {
      return "";
    }
    if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
      return "\"" + value.replace("\"", "\"\"") + "\"";
    }
    return value;
  }
}
