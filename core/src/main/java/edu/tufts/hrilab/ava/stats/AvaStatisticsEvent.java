/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025.
 */

package edu.tufts.hrilab.ava.stats;

import java.io.Serializable;

/**
 * Event DTO for Ava statistics notifications. Script-friendly: one callback signature,
 * filter by kind. Payload uses primitives and String only for TRADE serialization.
 */
public class AvaStatisticsEvent implements Serializable {

  private static final long serialVersionUID = 1L;

  public enum Kind {
    DOCKED,
    UNDOCKED,
    STOPPED_MOVING,
    PATH_EXECUTION_COMPLETED
  }

  private final Kind kind;
  private final long timestampMs;
  private final String dockName;
  private final Double cumulativeDistanceM;
  private final Long segmentStartTimestampMs;
  private final String driveStatusEnd;

  private AvaStatisticsEvent(Builder b) {
    this.kind = b.kind;
    this.timestampMs = b.timestampMs;
    this.dockName = b.dockName;
    this.cumulativeDistanceM = b.cumulativeDistanceM;
    this.segmentStartTimestampMs = b.segmentStartTimestampMs;
    this.driveStatusEnd = b.driveStatusEnd;
  }

  public Kind getKind() {
    return kind;
  }

  public long getTimestampMs() {
    return timestampMs;
  }

  public String getDockName() {
    return dockName;
  }

  public Double getCumulativeDistanceM() {
    return cumulativeDistanceM;
  }

  public Long getSegmentStartTimestampMs() {
    return segmentStartTimestampMs;
  }

  public String getDriveStatusEnd() {
    return driveStatusEnd;
  }

  public static Builder builder(Kind kind) {
    return new Builder(kind);
  }

  public static final class Builder {
    private final Kind kind;
    private long timestampMs = System.currentTimeMillis();
    private String dockName;
    private Double cumulativeDistanceM;
    private Long segmentStartTimestampMs;
    private String driveStatusEnd;

    public Builder(Kind kind) {
      this.kind = kind;
    }

    public Builder timestampMs(long timestampMs) {
      this.timestampMs = timestampMs;
      return this;
    }

    public Builder dockName(String dockName) {
      this.dockName = dockName;
      return this;
    }

    public Builder cumulativeDistanceM(double cumulativeDistanceM) {
      this.cumulativeDistanceM = cumulativeDistanceM;
      return this;
    }

    public Builder segmentStartTimestampMs(long segmentStartTimestampMs) {
      this.segmentStartTimestampMs = segmentStartTimestampMs;
      return this;
    }

    public Builder driveStatusEnd(String driveStatusEnd) {
      this.driveStatusEnd = driveStatusEnd;
      return this;
    }

    public AvaStatisticsEvent build() {
      return new AvaStatisticsEvent(this);
    }
  }
}
