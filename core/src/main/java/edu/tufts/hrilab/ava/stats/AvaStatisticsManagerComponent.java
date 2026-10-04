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
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDate;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

public class AvaStatisticsManagerComponent extends DiarcComponent {

  /** Statistic IDs exposed to dialogue; order matches list shown to user (option 1, 2, ...). */
  public static final String STAT_TIME_OFF_DOCK = "time_off_dock";
  public static final String STAT_DISTANCE_TRAVELED = "distance_traveled";
  public static final String STAT_TOURS_EXECUTED = "tours_executed";
  public static final String STAT_SCHEDULED_GOAL_EXECUTIONS = "scheduled_goal_executions";
  public static final String STAT_LOW_BATTERY_DOCK_EXECUTIONS = "low_battery_dock_executions";
  public static final String STAT_GOALS_COMPLETED = "goals_completed";
  public static final String STAT_GOALS_FAILED = "goals_failed";
  public static final String STAT_GOALS_CANCELED = "goals_canceled";
  public static final String STAT_AVERAGE_UNDOCKED_BATTERY = "average_undocked_battery";
  public static final String STAT_MEDIA_PLAYBACK_TOTAL = "media_playback_total";
  public static final String STAT_MEDIA_PLAYBACK_PHOTOS = "media_playback_photos";
  public static final String STAT_MEDIA_PLAYBACK_VIDEOS = "media_playback_videos";
  public static final String STAT_ESCORTS_COMPLETED = "escorts_completed";
  public static final String STAT_ESCORTS_FAILED = "escorts_failed";
  public static final String STAT_ESCORTS_CANCELED = "escorts_canceled";
  public static final String STAT_DELIVERIES_COMPLETED = "deliveries_completed";
  public static final String STAT_MOST_FREQUENTLY_FAILED = "most_frequently_failed";
  public static final String STAT_LEAST_FREQUENTLY_FAILED = "least_frequently_failed";
  public static final String STAT_MOST_FREQUENTLY_CANCELED = "most_frequently_canceled";
  public static final String STAT_MOST_VISITED_LOCATION = "most_visited_location";
  public static final String STAT_LOCATIONS_VISITED_COUNT = "locations_visited_count";
  public static final String STAT_MOST_FREQUENT_ESCORT_DESTINATION = "most_frequent_escort_destination";
  public static final String STAT_KIOSK_GOAL_TRIGGERS = "kiosk_goal_triggers";

  private static final Map<String, String> STAT_DISPLAY_NAMES = new LinkedHashMap<>();
  static {
    STAT_DISPLAY_NAMES.put(STAT_TIME_OFF_DOCK, "Time off dock");
    STAT_DISPLAY_NAMES.put(STAT_DISTANCE_TRAVELED, "Distance traveled");
    STAT_DISPLAY_NAMES.put(STAT_TOURS_EXECUTED, "Tours executed");
    STAT_DISPLAY_NAMES.put(STAT_SCHEDULED_GOAL_EXECUTIONS, "Scheduled goal executions");
    STAT_DISPLAY_NAMES.put(STAT_LOW_BATTERY_DOCK_EXECUTIONS, "Low-battery returns to dock");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_COMPLETED, "Goals completed");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_FAILED, "Goals failed");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_CANCELED, "Goals canceled");
    STAT_DISPLAY_NAMES.put(STAT_ESCORTS_COMPLETED, "Escorts completed");
    STAT_DISPLAY_NAMES.put(STAT_ESCORTS_FAILED, "Escorts failed");
    STAT_DISPLAY_NAMES.put(STAT_ESCORTS_CANCELED, "Escorts canceled");
    STAT_DISPLAY_NAMES.put(STAT_DELIVERIES_COMPLETED, "Deliveries completed");
    STAT_DISPLAY_NAMES.put(STAT_MOST_FREQUENTLY_FAILED, "Most frequently failed action");
    STAT_DISPLAY_NAMES.put(STAT_LEAST_FREQUENTLY_FAILED, "Least frequently failed action (that failed at least once)");
    STAT_DISPLAY_NAMES.put(STAT_MOST_FREQUENTLY_CANCELED, "Most frequently canceled action");
    STAT_DISPLAY_NAMES.put(STAT_MOST_VISITED_LOCATION, "Most visited location");
    STAT_DISPLAY_NAMES.put(STAT_LOCATIONS_VISITED_COUNT, "Number of locations visited");
    STAT_DISPLAY_NAMES.put(STAT_MOST_FREQUENT_ESCORT_DESTINATION, "Most frequent escort destination");
    STAT_DISPLAY_NAMES.put(STAT_KIOSK_GOAL_TRIGGERS, "Kiosk mode goal triggered");
    STAT_DISPLAY_NAMES.put(STAT_AVERAGE_UNDOCKED_BATTERY, "Average battery level when undocked");
    STAT_DISPLAY_NAMES.put(STAT_MEDIA_PLAYBACK_TOTAL, "Time spent playing media");
    STAT_DISPLAY_NAMES.put(STAT_MEDIA_PLAYBACK_PHOTOS, "Time spent on photos");
    STAT_DISPLAY_NAMES.put(STAT_MEDIA_PLAYBACK_VIDEOS, "Time spent on videos");
  }

  /** Category order and stat IDs per category for grouped option list. */
  private static final List<String> CATEGORY_ORDER = Arrays.asList("Mobility", "Goals", "Escorts & deliveries", "Drill-down & locations", "Media");
  private static final Map<String, List<String>> CATEGORY_TO_STATS = new LinkedHashMap<>();
  static {
    CATEGORY_TO_STATS.put("Mobility", Arrays.asList(STAT_TIME_OFF_DOCK, STAT_DISTANCE_TRAVELED));
    CATEGORY_TO_STATS.put("Goals", Arrays.asList(STAT_TOURS_EXECUTED, STAT_SCHEDULED_GOAL_EXECUTIONS, STAT_LOW_BATTERY_DOCK_EXECUTIONS, STAT_GOALS_COMPLETED, STAT_GOALS_FAILED, STAT_GOALS_CANCELED, STAT_KIOSK_GOAL_TRIGGERS));
    CATEGORY_TO_STATS.put("Escorts & deliveries", Arrays.asList(STAT_ESCORTS_COMPLETED, STAT_ESCORTS_FAILED, STAT_ESCORTS_CANCELED, STAT_DELIVERIES_COMPLETED));
    CATEGORY_TO_STATS.put("Drill-down & locations", Arrays.asList(STAT_MOST_FREQUENTLY_FAILED, STAT_LEAST_FREQUENTLY_FAILED, STAT_MOST_FREQUENTLY_CANCELED, STAT_MOST_VISITED_LOCATION, STAT_LOCATIONS_VISITED_COUNT, STAT_MOST_FREQUENT_ESCORT_DESTINATION));
    CATEGORY_TO_STATS.put("Media", Arrays.asList(STAT_MEDIA_PLAYBACK_TOTAL, STAT_MEDIA_PLAYBACK_PHOTOS, STAT_MEDIA_PLAYBACK_VIDEOS));
    // STAT_AVERAGE_UNDOCKED_BATTERY paused: current value is average of ad-hoc samples (UNDOCKED + STOPPED_MOVING only), not time-weighted. To fix: log battery at regular intervals while undocked, then average over those samples or use time-weighted formula.
  }

  private static final java.util.Set<String> BREAKDOWN_STAT_IDS = new java.util.LinkedHashSet<>(Arrays.asList(
      STAT_MOST_FREQUENTLY_FAILED, STAT_LEAST_FREQUENTLY_FAILED, STAT_MOST_FREQUENTLY_CANCELED,
      STAT_MOST_VISITED_LOCATION, STAT_LOCATIONS_VISITED_COUNT, STAT_MOST_FREQUENT_ESCORT_DESTINATION));

  /** Tour goal names contain this (case-insensitive) to count as "tour executed". */
  private static final String TOUR_GOAL_NAME_SUBSTRING = "tour";
  /** Escort goal names contain this (case-insensitive); e.g. escortToLocation, escortFromLocationToLocation. */
  private static final String ESCORT_GOAL_NAME_SUBSTRING = "escort";
  /** Delivery goal names contain this (case-insensitive) for deliveries_completed. */
  private static final String DELIVERY_GOAL_NAME_SUBSTRING = "delivery";
  /** Default kiosk goal name (KioskGoalManager default is chargeAtHome). Exact match so we don't count chargeAtHomeToPercent (battery monitor). TODO: generalize to count any kiosk-submitted goal (e.g. flag in CSV or match configured kiosk action). */
  private static final String KIOSK_GOAL_NAME_EXACT = "chargeAtHome";

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
    Predicate goalPredicate = (info.goal != null && info.goal.isPredicate()) ? (Predicate) info.goal : null;
    AvaStatisticsEvent event = AvaStatisticsEvent.builder(AvaStatisticsEvent.Kind.GOAL_STATUS)
        .timestampMs(System.currentTimeMillis())
        .goalGid(info.gid)
        .goalName(goalName)
        .goalStatus(info.status.name())
        .goalStartMs(info.start >= 0 ? Long.valueOf(info.start) : null)
        .goalEndMs(info.end >= 0 ? info.end : null)
        .scheduled(scheduled)
        .urgentGoHome(urgentGoHome)
        .goalPredicate(goalPredicate)
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

  /**
   * Records an event to the statistics CSV. Any component (e.g. AvaDisplayComponent) can call this
   * to log MEDIA_PLAYBACK or other events without registering with AvaComponent.
   */
  @TRADEService
  public void recordAvaStatisticsEvent(AvaStatisticsEvent event) {
    onAvaStatisticsEvent(event);
  }

  @TRADEService
  public void onAvaStatisticsEvent(AvaStatisticsEvent event) {
    if (writer == null) {
      return;
    }
    writeLock.lock();
    try {
      if (!headerWritten) {
        // Order must match CSV_IDX_* and all query methods that parse cols[] (0=timestamp_ms, 1=kind, 3=cumulative_distance_m, 7=goal_name, 8=goal_status, 11=scheduled, 12=urgent_go_home, 13=goal_predicate, 14=battery_pct, 15=media_type, 16=duration_ms).
        writer.write("timestamp_ms,kind,dock_name,cumulative_distance_m,segment_start_timestamp_ms,drive_status_end,goal_gid,goal_name,goal_status,goal_start_ms,goal_end_ms,scheduled,urgent_go_home,goal_predicate,battery_pct,media_type,duration_ms");
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
        row.append(',').append(escapeCsv(event.getGoalPredicate() != null ? event.getGoalPredicate().toString() : ""));
      } else {
        row.append(",,,,,,,,");
      }
      row.append(',').append(event.getBatteryPct() != null ? event.getBatteryPct() : "");
      row.append(',').append(event.getMediaType() != null ? escapeCsv(event.getMediaType()) : "");
      row.append(',').append(event.getDurationMs() != null ? event.getDurationMs() : "");
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

  // -------------------------------------------------------------------------
  // Query API for "discuss robot performance" dialogue
  // -------------------------------------------------------------------------

  /**
   * Returns statistic IDs in display order (option 1, 2, ... for list+option flow),
   * grouped by category so ASL can list by section.
   */
  @TRADEService
  public List<String> getAvailableStatisticIds() {
    List<String> out = new ArrayList<>();
    for (String category : CATEGORY_ORDER) {
      List<String> ids = CATEGORY_TO_STATS.get(category);
      if (ids != null) out.addAll(ids);
    }
    return out;
  }

  /** Category names in the order they should be announced (Mobility, Goals, Battery, Media). */
  @TRADEService
  public List<String> getCategoryNamesInOrder() {
    return new ArrayList<>(CATEGORY_ORDER);
  }

  /** Statistic IDs in the given category (for grouped listing). */
  @TRADEService
  public List<String> getStatisticIdsInCategory(String category) {
    List<String> ids = CATEGORY_TO_STATS.get(category);
    return ids != null ? new ArrayList<>(ids) : Collections.emptyList();
  }

  @TRADEService
  public String getStatisticDisplayName(String statisticId) {
    return STAT_DISPLAY_NAMES.getOrDefault(statisticId, statisticId);
  }

  /**
   * Builds a duration predicate for the language system, following the same (number, unit) pattern
   * as period/start in scheduling: duration(days(D), hours(H), minutes(M), seconds(S)).
   */
  private static Predicate createDurationValueTerm(double totalMinutes) {
    long totalSeconds = totalMinutes < 0 || Double.isNaN(totalMinutes) ? 0 : Math.round(totalMinutes * 60);
    long days = totalSeconds / 86400;
    long hours = (totalSeconds % 86400) / 3600;
    long minutes = (totalSeconds % 3600) / 60;
    long seconds = totalSeconds % 60;
    Predicate daysPred = Factory.createPredicate("days", Factory.createSymbol(String.valueOf(days)));
    Predicate hoursPred = Factory.createPredicate("hours", Factory.createSymbol(String.valueOf(hours)));
    Predicate minutesPred = Factory.createPredicate("minutes", Factory.createSymbol(String.valueOf(minutes)));
    Predicate secondsPred = Factory.createPredicate("seconds", Factory.createSymbol(String.valueOf(seconds)));
    return Factory.createPredicate("duration", daysPred, hoursPred, minutesPred, secondsPred);
  }

  /**
   * Builds the value term for a statistic report: duration(...) for time_off_dock,
   * distance(value, meters) for distance_traveled, count(N) for counts. Matches AVA patterns
   * (period(number, timeUnit), etc.).
   */
  @TRADEService
  public Symbol createStatisticValueTerm(String statisticId, double value) {
    if (statisticId == null) {
      return Factory.createPredicate("count", Factory.createSymbol(String.valueOf((int) Math.round(value))));
    }
    switch (statisticId) {
      case STAT_TIME_OFF_DOCK:
        return createDurationValueTerm(value);
      case STAT_DISTANCE_TRAVELED:
        return Factory.createPredicate("distance", Factory.createSymbol(String.format("%.1f", value)), Factory.createSymbol("meters"));
      case STAT_AVERAGE_UNDOCKED_BATTERY:
        return Factory.createPredicate("percent", Factory.createSymbol(String.format("%.1f", value)));
      case STAT_MEDIA_PLAYBACK_TOTAL:
      case STAT_MEDIA_PLAYBACK_PHOTOS:
      case STAT_MEDIA_PLAYBACK_VIDEOS:
        return createDurationValueTerm(value);
      case STAT_TOURS_EXECUTED:
      case STAT_SCHEDULED_GOAL_EXECUTIONS:
      case STAT_LOW_BATTERY_DOCK_EXECUTIONS:
      case STAT_GOALS_COMPLETED:
      case STAT_GOALS_FAILED:
      case STAT_GOALS_CANCELED:
      case STAT_ESCORTS_COMPLETED:
      case STAT_ESCORTS_FAILED:
      case STAT_ESCORTS_CANCELED:
      case STAT_DELIVERIES_COMPLETED:
      case STAT_KIOSK_GOAL_TRIGGERS:
      default:
        return Factory.createPredicate("count", Factory.createSymbol(String.valueOf((int) Math.round(value))));
    }
  }

  /**
   * Builds the full statisticsReport predicate for the language system: statisticsReport(statId, startDate, endDate, valueTerm)
   * with valueTerm as a structured predicate (duration(...), distance(...), count(...)).
   */
  @TRADEService
  public Predicate createStatisticsReportPredicate(String statisticId, String startDateStr, String endDateStr, double value) {
    Symbol valueTerm = createStatisticValueTerm(statisticId, value);
    return createStatisticsReportPredicateWithValueTerm(statisticId, startDateStr, endDateStr, valueTerm);
  }

  /**
   * Builds statisticsReport with an arbitrary value term (e.g. mostFailedGoals(goal1, goal2, count(N)) for ties; last arg is always count).
   * Use so RR can resolve refs inside the value term when generateResponse(predicate) is used.
   */
  @TRADEService
  public Predicate createStatisticsReportPredicateWithValueTerm(String statisticId, String startDateStr, String endDateStr, Symbol valueTerm) {
    return Factory.createPredicate("statisticsReport",
        Factory.createSymbol(statisticId),
        Factory.createSymbol(startDateStr),
        Factory.createSymbol(endDateStr),
        valueTerm);
  }

  /**
   * Returns the report predicate for the given statistic over the period, for use with generateResponse.
   * For numeric stats: queries the value and builds the report predicate. For drill-down/location stats: builds the report predicate from aggregated data.
   * Returns null if no data for that statistic in that period.
   */
  @TRADEService
  public Predicate getStatisticsReportPredicate(String statisticId, long startMs, long endMs, String startDateStr, String endDateStr) {
    if (statisticId == null || !STAT_DISPLAY_NAMES.containsKey(statisticId)) {
      return null;
    }
    if (BREAKDOWN_STAT_IDS.contains(statisticId)) {
      return getStatisticsReportPredicateForBreakdown(statisticId, startMs, endMs, startDateStr, endDateStr);
    }
    double value = queryStatistics(statisticId, startMs, endMs);
    return createStatisticsReportPredicate(statisticId, startDateStr, endDateStr, value);
  }

  /** Returns the report predicate for drill-down/location stats, or null if no data. */
  private Predicate getStatisticsReportPredicateForBreakdown(String statisticId, long startMs, long endMs, String startDateStr, String endDateStr) {
    if (statisticId == null || !BREAKDOWN_STAT_IDS.contains(statisticId)) {
      return null;
    }
    switch (statisticId) {
      case STAT_MOST_FREQUENTLY_FAILED:
        return getMostFailedGoalReport(startMs, endMs, startDateStr, endDateStr);
      case STAT_LEAST_FREQUENTLY_FAILED:
        return getLeastFailedGoalReport(startMs, endMs, startDateStr, endDateStr);
      case STAT_MOST_FREQUENTLY_CANCELED:
        return getMostCanceledGoalReport(startMs, endMs, startDateStr, endDateStr);
      case STAT_MOST_VISITED_LOCATION:
        return getMostVisitedLocationReport(startMs, endMs, startDateStr, endDateStr);
      case STAT_LOCATIONS_VISITED_COUNT:
        return getLocationsVisitedCountReport(startMs, endMs, startDateStr, endDateStr);
      case STAT_MOST_FREQUENT_ESCORT_DESTINATION:
        return getMostFrequentEscortDestinationReport(startMs, endMs, startDateStr, endDateStr);
      default:
        return null;
    }
  }

  /**
   * Parses a date string (YYYY-MM-DD) to start-of-day epoch ms in system default timezone.
   * Inclusive start for analysis period.
   */
  @TRADEService
  public long parseDateToStartMs(String dateStr) {
    LocalDate d = LocalDate.parse(dateStr.trim(), DateTimeFormatter.ISO_LOCAL_DATE);
    return ZonedDateTime.of(d.atStartOfDay(), ZoneId.systemDefault()).toInstant().toEpochMilli();
  }

  /**
   * Parses a date string (YYYY-MM-DD) to end-of-day epoch ms in system default timezone.
   * Inclusive end for analysis period.
   */
  @TRADEService
  public long parseDateToEndMs(String dateStr) {
    LocalDate d = LocalDate.parse(dateStr.trim(), DateTimeFormatter.ISO_LOCAL_DATE);
    return ZonedDateTime.of(d.plusDays(1).atStartOfDay(), ZoneId.systemDefault()).toInstant().toEpochMilli() - 1;
  }

  /**
   * Queries the statistics CSV for the given statistic over [startMs, endMs] (inclusive).
   * Returns a numeric value: counts as whole number (e.g. 10.0), time off dock in minutes,
   * distance in meters, average battery 0-100, media playback in minutes.
   */
  @TRADEService
  public double queryStatistics(String statisticId, long startMs, long endMs) {
    if (statisticId == null || !STAT_DISPLAY_NAMES.containsKey(statisticId)) {
      log.warn("[queryStatistics] Unknown statistic id: {}", statisticId);
      return 0;
    }
    if (!Files.exists(statisticsFile)) {
      return 0;
    }
    switch (statisticId) {
      case STAT_TIME_OFF_DOCK:
        return queryTimeOffDockMinutes(startMs, endMs);
      case STAT_DISTANCE_TRAVELED:
        return queryDistanceTraveledMeters(startMs, endMs);
      case STAT_TOURS_EXECUTED:
        return queryToursExecuted(startMs, endMs);
      case STAT_SCHEDULED_GOAL_EXECUTIONS:
        return queryScheduledGoalExecutions(startMs, endMs);
      case STAT_LOW_BATTERY_DOCK_EXECUTIONS:
        return queryLowBatteryDockExecutions(startMs, endMs);
      case STAT_GOALS_COMPLETED:
        return queryGoalStatusCount(startMs, endMs, "SUCCEEDED");
      case STAT_GOALS_FAILED:
        return queryGoalStatusCount(startMs, endMs, "FAILED");
      case STAT_GOALS_CANCELED:
        return queryGoalStatusCount(startMs, endMs, "CANCELED");
      case STAT_AVERAGE_UNDOCKED_BATTERY:
        return queryAverageUndockedBattery(startMs, endMs);
      case STAT_MEDIA_PLAYBACK_TOTAL:
        return queryMediaPlaybackMinutes(startMs, endMs, null);
      case STAT_MEDIA_PLAYBACK_PHOTOS:
        return queryMediaPlaybackMinutes(startMs, endMs, "image");
      case STAT_MEDIA_PLAYBACK_VIDEOS:
        return queryMediaPlaybackMinutes(startMs, endMs, "video");
      case STAT_ESCORTS_COMPLETED:
        return queryGoalStatusCountWithFilter(startMs, endMs, "SUCCEEDED", name ->
            name != null && name.toLowerCase().contains(ESCORT_GOAL_NAME_SUBSTRING));
      case STAT_ESCORTS_FAILED:
        return queryGoalStatusCountWithFilter(startMs, endMs, "FAILED", name ->
            name != null && name.toLowerCase().contains(ESCORT_GOAL_NAME_SUBSTRING));
      case STAT_ESCORTS_CANCELED:
        return queryGoalStatusCountWithFilter(startMs, endMs, "CANCELED", name ->
            name != null && name.toLowerCase().contains(ESCORT_GOAL_NAME_SUBSTRING));
      case STAT_DELIVERIES_COMPLETED:
        return queryGoalStatusCountWithFilter(startMs, endMs, "SUCCEEDED", name ->
            name != null && name.toLowerCase().contains(DELIVERY_GOAL_NAME_SUBSTRING));
      case STAT_KIOSK_GOAL_TRIGGERS:
        return queryDistinctGoalCountByExactName(startMs, endMs, KIOSK_GOAL_NAME_EXACT);
      default:
        return 0;
    }
  }

  private double queryTimeOffDockMinutes(long startMs, long endMs) {
    List<long[]> undockedIntervals = new ArrayList<>();
    Long lastUndockedMs = null;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      String header = r.readLine();
      if (header == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 2) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < 0) continue;
        String kind = cols[1].trim();
        if (ts < startMs || ts > endMs) {
          if ("DOCKED".equals(kind) && lastUndockedMs != null) {
            lastUndockedMs = null;
          }
          continue;
        }
        if ("UNDOCKED".equals(kind)) {
          lastUndockedMs = ts;
        } else if ("DOCKED".equals(kind) && lastUndockedMs != null) {
          undockedIntervals.add(new long[] { lastUndockedMs, ts });
          lastUndockedMs = null;
        }
      }
      if (lastUndockedMs != null) {
        // Open interval: still off dock. Cap at "now" so we don't count future time.
        long endCapped = Math.min(endMs, System.currentTimeMillis());
        undockedIntervals.add(new long[] { lastUndockedMs, endCapped });
      }
    } catch (IOException e) {
      log.error("[queryTimeOffDockMinutes] Error reading statistics file", e);
      return 0;
    }
    long totalMs = 0;
    for (long[] iv : undockedIntervals) {
      totalMs += (iv[1] - iv[0]);
    }
    return totalMs / (60_000.0);
  }

  private double queryDistanceTraveledMeters(long startMs, long endMs) {
    double sum = 0;
    int segmentCount = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 4) continue;
        if (!"STOPPED_MOVING".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts >= startMs && ts <= endMs) {
          double m = parseDouble(cols[3], 0);
          sum += m;
          segmentCount++;
        }
      }
    } catch (IOException e) {
      log.error("[queryDistanceTraveledMeters] Error reading statistics file", e);
      return 0;
    }
    if (log.isDebugEnabled()) {
      log.debug("[queryDistanceTraveledMeters] range [{}, {}] -> {} m ({} segments)",
          startMs, endMs, sum, segmentCount);
    }
    return sum;
  }

  private double queryToursExecuted(long startMs, long endMs) {
    return queryGoalStatusCountWithFilter(startMs, endMs, "SUCCEEDED", name ->
        name != null && name.toLowerCase().contains(TOUR_GOAL_NAME_SUBSTRING));
  }

  private double queryScheduledGoalExecutions(long startMs, long endMs) {
    int count = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 13) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        if (!"SUCCEEDED".equals(cols[8].trim())) continue;
        if ("true".equalsIgnoreCase(cols[11].trim())) count++;
      }
    } catch (IOException e) {
      log.error("[queryScheduledGoalExecutions] Error reading statistics file", e);
      return 0;
    }
    return count;
  }

  private double queryLowBatteryDockExecutions(long startMs, long endMs) {
    int count = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 13) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts >= startMs && ts <= endMs && "true".equalsIgnoreCase(cols[12].trim())) count++;
      }
    } catch (IOException e) {
      log.error("[queryLowBatteryDockExecutions] Error reading statistics file", e);
      return 0;
    }
    return count;
  }

  /** Expected CSV column count; must match header and onAvaStatisticsEvent write order. */
  private static final int CSV_COLUMN_COUNT = 17;
  /** Column index for goal_predicate in CSV. */
  private static final int CSV_IDX_GOAL_PREDICATE = 13;
  /** Column index for battery_pct in CSV (after goal columns). */
  private static final int CSV_IDX_BATTERY_PCT = 14;
  /** Column indices for MEDIA_PLAYBACK. */
  private static final int CSV_IDX_MEDIA_TYPE = 15;
  private static final int CSV_IDX_DURATION_MS = 16;

  private double queryAverageUndockedBattery(long startMs, long endMs) {
    double sum = 0;
    int n = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length <= CSV_IDX_BATTERY_PCT) continue;
        String kind = cols[1].trim();
        if (!"UNDOCKED".equals(kind) && !"STOPPED_MOVING".equals(kind)) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        double pct = parseDouble(cols[CSV_IDX_BATTERY_PCT], -1);
        if (pct >= 0 && pct <= 100) {
          sum += pct;
          n++;
        }
      }
    } catch (IOException e) {
      log.error("[queryAverageUndockedBattery] Error reading statistics file", e);
      return 0;
    }
    return n == 0 ? 0 : sum / n;
  }

  /**
   * Sum of MEDIA_PLAYBACK duration in the range, in minutes. If filter is null, sum all; if "image", sum image only; if "video", sum video only.
   */
  private double queryMediaPlaybackMinutes(long startMs, long endMs, String filterMediaType) {
    long totalMs = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length <= CSV_IDX_DURATION_MS) continue;
        if (!"MEDIA_PLAYBACK".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        if (filterMediaType != null && !filterMediaType.equalsIgnoreCase(cols[CSV_IDX_MEDIA_TYPE].trim())) continue;
        long dur = parseLong(cols[CSV_IDX_DURATION_MS], 0);
        if (dur > 0) totalMs += dur;
      }
    } catch (IOException e) {
      log.error("[queryMediaPlaybackMinutes] Error reading statistics file", e);
      return 0;
    }
    return totalMs / (60_000.0);
  }

  private double queryGoalStatusCount(long startMs, long endMs, String status) {
    int count = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 9) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts >= startMs && ts <= endMs && status.equals(cols[8].trim())) count++;
      }
    } catch (IOException e) {
      log.error("[queryGoalStatusCount] Error reading statistics file", e);
      return 0;
    }
    return count;
  }

  @FunctionalInterface
  private interface NameFilter {
    boolean test(String goalName);
  }

  private double queryGoalStatusCountWithFilter(long startMs, long endMs, String status, NameFilter nameFilter) {
    int count = 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 9) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        if (!status.equals(cols[8].trim())) continue;
        String goalName = cols.length > 7 ? unescapeCsv(cols[7]) : null;
        if (nameFilter.test(goalName)) count++;
      }
    } catch (IOException e) {
      log.error("[queryGoalStatusCountWithFilter] Error reading statistics file", e);
      return 0;
    }
    return count;
  }

  /** Count of distinct goals (by goal_gid) whose goal_name equals exactName. Used for kiosk trigger count (default chargeAtHome; excludes chargeAtHomeToPercent). */
  private double queryDistinctGoalCountByExactName(long startMs, long endMs, String exactName) {
    java.util.Set<Long> gids = new java.util.HashSet<>();
    if (exactName == null) return 0;
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return 0;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length < 8) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        String goalName = unescapeCsv(cols[7]);
        if (!exactName.equals(goalName)) continue;
        long gid = parseLong(cols[6], -1);
        if (gid >= 0) gids.add(gid);
      }
    } catch (IOException e) {
      log.error("[queryDistinctGoalCountByExactName] Error reading statistics file", e);
      return 0;
    }
    return gids.size();
  }

  /** Aggregates GOAL_STATUS rows by goal_predicate (col 13) for the given status. Returns map of goalPredicateStr -> count. */
  private Map<String, Long> aggregateGoalPredicateByStatus(long startMs, long endMs, String status) {
    Map<String, Long> counts = new LinkedHashMap<>();
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return counts;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length <= CSV_IDX_GOAL_PREDICATE) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        if (!status.equals(cols[8].trim())) continue;
        String gp = unescapeCsv(cols[CSV_IDX_GOAL_PREDICATE]);
        if (gp == null || gp.isEmpty()) continue;
        counts.merge(gp, 1L, Long::sum);
      }
    } catch (IOException e) {
      log.error("[aggregateGoalPredicateByStatus] Error reading statistics file", e);
    }
    return counts;
  }

  /** Returns [goalPredicateStr, count] for the entry with max (or min if max=false) count, or null if empty. */
  /** Returns all keys tied for the best count (max or min). Returns null if empty. Caller can get count from counts.get(keys.get(0)). */
  private static List<String> findAllTiedByCount(Map<String, Long> counts, boolean max) {
    if (counts == null || counts.isEmpty()) return null;
    long best = max ? Long.MIN_VALUE : Long.MAX_VALUE;
    List<String> bestKeys = new ArrayList<>();
    for (Map.Entry<String, Long> e : counts.entrySet()) {
      long c = e.getValue();
      if (max ? c > best : c < best) {
        best = c;
        bestKeys.clear();
        bestKeys.add(e.getKey());
      } else if (c == best) {
        bestKeys.add(e.getKey());
      }
    }
    return bestKeys.isEmpty() ? null : bestKeys;
  }

  private static Predicate parseGoalPredicateSafe(String s) {
    if (s == null || s.isEmpty()) return null;
    try {
      return Factory.createPredicate(s.trim());
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Extract the location ref (destination) from a goal predicate string for "most visited" / escort stats.
   * goToLocation: destination is always the last arg — (location), (actor, location), or (actor, start, end).
   * escortToLocation(actor, location): destination = arg 1.
   * escortFromLocationToLocation(actor, from, to): destination = arg 2 (to).
   */
  private static Symbol extractLocationRefFromGoalPredicateString(String goalPredicateStr) {
    Predicate p = parseGoalPredicateSafe(goalPredicateStr);
    if (p == null || p.getArgs() == null || p.getArgs().isEmpty()) return null;
    String name = p.getName();
    int idx = -1;
    if ("goToLocation".equals(name)) {
      idx = p.getArgs().size() - 1;  // destination is last: (loc), (actor, loc), or (actor, start, end)
    } else if ("escortToLocation".equals(name)) idx = 1;
    else if ("escortFromLocationToLocation".equals(name)) idx = 2;
    if (idx < 0 || idx >= p.getArgs().size()) return null;
    Symbol arg = p.get(idx);
    if (arg == null || arg.isVariable()) return null;
    return arg;
  }

  /** Aggregates GOAL_STATUS rows by extracted location ref (for goToLocation / escort goals). escortOnly: only goal_name contains "escort". */
  private Map<String, Long> aggregateLocationRefs(long startMs, long endMs, boolean escortOnly) {
    Map<String, Long> counts = new LinkedHashMap<>();
    try (BufferedReader r = Files.newBufferedReader(statisticsFile, StandardCharsets.UTF_8)) {
      if (r.readLine() == null) return counts;
      String line;
      while ((line = r.readLine()) != null) {
        String[] cols = parseCsvLine(line);
        if (cols.length <= CSV_IDX_GOAL_PREDICATE) continue;
        if (!"GOAL_STATUS".equals(cols[1].trim())) continue;
        long ts = parseLong(cols[0], -1);
        if (ts < startMs || ts > endMs) continue;
        String goalName = cols.length > 7 ? unescapeCsv(cols[7]) : null;
        if (escortOnly && (goalName == null || !goalName.toLowerCase().contains(ESCORT_GOAL_NAME_SUBSTRING))) continue;
        String gp = unescapeCsv(cols[CSV_IDX_GOAL_PREDICATE]);
        Symbol ref = extractLocationRefFromGoalPredicateString(gp);
        if (ref == null) continue;
        counts.merge(ref.toString(), 1L, Long::sum);
      }
    } catch (IOException e) {
      log.error("[aggregateLocationRefs] Error reading statistics file", e);
    }
    return counts;
  }

  /** Builds value term with all tied items (goals or location refs) and count; predicate name plural (e.g. mostFailedGoals). */
  private static Symbol buildTiedReportValueTerm(String predicateName, List<Symbol> tiedRefs, long count) {
    if (tiedRefs == null || tiedRefs.isEmpty()) return null;
    List<Symbol> args = new ArrayList<>(tiedRefs);
    args.add(Factory.createPredicate("count", Factory.createSymbol(String.valueOf(count))));
    return Factory.createPredicate(predicateName, args);
  }

  @TRADEService
  public Predicate getMostFailedGoalReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateGoalPredicateByStatus(startMs, endMs, "FAILED");
    List<String> tied = findAllTiedByCount(counts, true);
    if (tied == null || tied.isEmpty()) return null;
    long countVal = counts.get(tied.get(0));
    List<Symbol> goalPreds = new ArrayList<>();
    for (String key : tied) {
      Predicate p = parseGoalPredicateSafe(key);
      if (p != null) goalPreds.add(p);
    }
    if (goalPreds.isEmpty()) return null;
    Symbol valueTerm = buildTiedReportValueTerm("mostFailedGoals", goalPreds, countVal);
    return createStatisticsReportPredicateWithValueTerm("most_frequently_failed", startDateStr, endDateStr, valueTerm);
  }

  @TRADEService
  public Predicate getLeastFailedGoalReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateGoalPredicateByStatus(startMs, endMs, "FAILED");
    List<String> tied = findAllTiedByCount(counts, false);
    if (tied == null || tied.isEmpty()) return null;
    long countVal = counts.get(tied.get(0));
    List<Symbol> goalPreds = new ArrayList<>();
    for (String key : tied) {
      Predicate p = parseGoalPredicateSafe(key);
      if (p != null) goalPreds.add(p);
    }
    if (goalPreds.isEmpty()) return null;
    Symbol valueTerm = buildTiedReportValueTerm("leastFailedGoals", goalPreds, countVal);
    return createStatisticsReportPredicateWithValueTerm("least_frequently_failed", startDateStr, endDateStr, valueTerm);
  }

  @TRADEService
  public Predicate getMostCanceledGoalReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateGoalPredicateByStatus(startMs, endMs, "CANCELED");
    List<String> tied = findAllTiedByCount(counts, true);
    if (tied == null || tied.isEmpty()) return null;
    long countVal = counts.get(tied.get(0));
    List<Symbol> goalPreds = new ArrayList<>();
    for (String key : tied) {
      Predicate p = parseGoalPredicateSafe(key);
      if (p != null) goalPreds.add(p);
    }
    if (goalPreds.isEmpty()) return null;
    Symbol valueTerm = buildTiedReportValueTerm("mostCanceledGoals", goalPreds, countVal);
    return createStatisticsReportPredicateWithValueTerm("most_frequently_canceled", startDateStr, endDateStr, valueTerm);
  }

  @TRADEService
  public Predicate getMostVisitedLocationReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateLocationRefs(startMs, endMs, false);
    List<String> tied = findAllTiedByCount(counts, true);
    if (tied == null || tied.isEmpty()) return null;
    long countVal = counts.get(tied.get(0));
    List<Symbol> locRefs = new ArrayList<>();
    for (String key : tied) locRefs.add(Factory.createSymbol(key));
    Symbol valueTerm = buildTiedReportValueTerm("mostVisitedLocations", locRefs, countVal);
    return createStatisticsReportPredicateWithValueTerm("most_visited_location", startDateStr, endDateStr, valueTerm);
  }

  @TRADEService
  public Predicate getLocationsVisitedCountReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateLocationRefs(startMs, endMs, false);
    int n = counts.size();
    Symbol valueTerm = Factory.createPredicate("count", Factory.createSymbol(String.valueOf(n)));
    return createStatisticsReportPredicateWithValueTerm("locations_visited_count", startDateStr, endDateStr, valueTerm);
  }

  @TRADEService
  public Predicate getMostFrequentEscortDestinationReport(long startMs, long endMs, String startDateStr, String endDateStr) {
    Map<String, Long> counts = aggregateLocationRefs(startMs, endMs, true);
    List<String> tied = findAllTiedByCount(counts, true);
    if (tied == null || tied.isEmpty()) return null;
    long countVal = counts.get(tied.get(0));
    List<Symbol> locRefs = new ArrayList<>();
    for (String key : tied) locRefs.add(Factory.createSymbol(key));
    Symbol valueTerm = buildTiedReportValueTerm("mostFrequentEscortDestinations", locRefs, countVal);
    return createStatisticsReportPredicateWithValueTerm("most_frequent_escort_destination", startDateStr, endDateStr, valueTerm);
  }

  private static long parseLong(String s, long def) {
    if (s == null || s.trim().isEmpty()) return def;
    try {
      return Long.parseLong(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static double parseDouble(String s, double def) {
    if (s == null || s.trim().isEmpty()) return def;
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  /** Simple CSV line parse: no quoted commas inside fields for numeric/kind columns we use. */
  private static String[] parseCsvLine(String line) {
    List<String> out = new ArrayList<>();
    StringBuilder cur = new StringBuilder();
    boolean inQuotes = false;
    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == '"') {
        inQuotes = !inQuotes;
      } else if ((c == ',' && !inQuotes) || c == '\n') {
        out.add(cur.toString().trim());
        cur = new StringBuilder();
      } else {
        cur.append(c);
      }
    }
    out.add(cur.toString().trim());
    return out.toArray(new String[0]);
  }

  private static String unescapeCsv(String value) {
    if (value == null) return null;
    if (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
      return value.substring(1, value.length() - 1).replace("\"\"", "\"");
    }
    return value;
  }
}
