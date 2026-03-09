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
  public static final String STAT_PANIC_DOCK_EXECUTIONS = "panic_dock_executions";
  public static final String STAT_GOALS_COMPLETED = "goals_completed";
  public static final String STAT_GOALS_FAILED = "goals_failed";
  public static final String STAT_GOALS_CANCELED = "goals_canceled";

  private static final Map<String, String> STAT_DISPLAY_NAMES = new LinkedHashMap<>();
  static {
    STAT_DISPLAY_NAMES.put(STAT_TIME_OFF_DOCK, "Time off dock");
    STAT_DISPLAY_NAMES.put(STAT_DISTANCE_TRAVELED, "Distance traveled");
    STAT_DISPLAY_NAMES.put(STAT_TOURS_EXECUTED, "Tours executed");
    STAT_DISPLAY_NAMES.put(STAT_SCHEDULED_GOAL_EXECUTIONS, "Scheduled goal executions");
    STAT_DISPLAY_NAMES.put(STAT_PANIC_DOCK_EXECUTIONS, "Panic dock (urgent go home) executions");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_COMPLETED, "Goals completed");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_FAILED, "Goals failed");
    STAT_DISPLAY_NAMES.put(STAT_GOALS_CANCELED, "Goals canceled");
  }

  /** Tour goal names contain this (case-insensitive) to count as "tour executed". */
  private static final String TOUR_GOAL_NAME_SUBSTRING = "tour";

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

  // -------------------------------------------------------------------------
  // Query API for "discuss robot performance" dialogue
  // -------------------------------------------------------------------------

  /**
   * Returns statistic IDs in display order (option 1, 2, ... for list+option flow).
   * Future work (media, locations, etc.): see docs/AVA_STATISTICS.md.
   */
  @TRADEService
  public List<String> getAvailableStatisticIds() {
    return new ArrayList<>(STAT_DISPLAY_NAMES.keySet());
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
      case STAT_TOURS_EXECUTED:
      case STAT_SCHEDULED_GOAL_EXECUTIONS:
      case STAT_PANIC_DOCK_EXECUTIONS:
      case STAT_GOALS_COMPLETED:
      case STAT_GOALS_FAILED:
      case STAT_GOALS_CANCELED:
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
    return Factory.createPredicate("statisticsReport",
        Factory.createSymbol(statisticId),
        Factory.createSymbol(startDateStr),
        Factory.createSymbol(endDateStr),
        valueTerm);
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
   * distance in meters.
   *
   * TODO: Add statistics for media playback (count and image vs video breakdown); requires
   * event source for media playback (subgoal or media component), not only top-level GOAL_STATUS.
   * See docs/AVA_STATISTICS.md for full future-work list.
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
      case STAT_PANIC_DOCK_EXECUTIONS:
        return queryPanicDockExecutions(startMs, endMs);
      case STAT_GOALS_COMPLETED:
        return queryGoalStatusCount(startMs, endMs, "SUCCEEDED");
      case STAT_GOALS_FAILED:
        return queryGoalStatusCount(startMs, endMs, "FAILED");
      case STAT_GOALS_CANCELED:
        return queryGoalStatusCount(startMs, endMs, "CANCELED");
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

  private double queryPanicDockExecutions(long startMs, long endMs) {
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
      log.error("[queryPanicDockExecutions] Error reading statistics file", e);
      return 0;
    }
    return count;
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
