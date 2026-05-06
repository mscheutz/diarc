/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.ava.stats;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
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
      log.error("Could not create statistics directory: {}", statisticsDir, e);
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
      log.error("Could not open statistics file: {}", statisticsFile, e);
    }
  }

  @Override
  protected void postRegistrationInit() {
    try {
      TRADE.registerNotificationNowOrWhenJoined(this,
          new TRADEServiceConstraints().name("registerForAvaStatisticsNotifications").argTypes(TRADEServiceInfo.class),
          "registerAvaStatisticsCallback");
    } catch (TRADEException e) {
      log.error("Error registering for Ava statistics notifications.", e);
    }
  }

  @TRADEService
  public void registerAvaStatisticsCallback(TRADEServiceInfo service) {
    TRADEServiceInfo myService = getMyService("onAvaStatisticsEvent", AvaStatisticsEvent.class);
    try {
      service.call(void.class, myService);
    } catch (TRADEException e) {
      log.error("Error registering callback with Ava statistics producer.", e);
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
        writer.write("timestamp_ms,kind,dock_name,cumulative_distance_m,segment_start_timestamp_ms,drive_status_end");
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
      writer.write(row.toString());
      writer.newLine();
      writer.flush();
    } catch (IOException e) {
      log.error("Error writing statistics event.", e);
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
