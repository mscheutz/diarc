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
    GOAL_STATUS
  }

  /** Goal name used for "urgent go home" (panic dock) identification. */
  public static final String URGENT_GO_HOME_GOAL_NAME = "urgentGoHomeAndChargeToPercent";

  private final Kind kind;
  private final long timestampMs;
  private final String dockName;
  private final Double cumulativeDistanceM;
  private final Long segmentStartTimestampMs;
  private final String driveStatusEnd;
  // Goal event fields (non-null only when kind == GOAL_STATUS)
  private final Long goalGid;
  private final String goalName;
  private final String goalStatus;
  private final Long goalStartMs;
  private final Long goalEndMs;
  private final Boolean scheduled;
  private final Boolean urgentGoHome;

  private AvaStatisticsEvent(Builder b) {
    this.kind = b.kind;
    this.timestampMs = b.timestampMs;
    this.dockName = b.dockName;
    this.cumulativeDistanceM = b.cumulativeDistanceM;
    this.segmentStartTimestampMs = b.segmentStartTimestampMs;
    this.driveStatusEnd = b.driveStatusEnd;
    this.goalGid = b.goalGid;
    this.goalName = b.goalName;
    this.goalStatus = b.goalStatus;
    this.goalStartMs = b.goalStartMs;
    this.goalEndMs = b.goalEndMs;
    this.scheduled = b.scheduled;
    this.urgentGoHome = b.urgentGoHome;
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

  public Long getGoalGid() {
    return goalGid;
  }

  public String getGoalName() {
    return goalName;
  }

  public String getGoalStatus() {
    return goalStatus;
  }

  public Long getGoalStartMs() {
    return goalStartMs;
  }

  public Long getGoalEndMs() {
    return goalEndMs;
  }

  public Boolean getScheduled() {
    return scheduled;
  }

  public Boolean getUrgentGoHome() {
    return urgentGoHome;
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
    private Long goalGid;
    private String goalName;
    private String goalStatus;
    private Long goalStartMs;
    private Long goalEndMs;
    private Boolean scheduled;
    private Boolean urgentGoHome;

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

    public Builder goalGid(long goalGid) {
      this.goalGid = goalGid;
      return this;
    }

    public Builder goalName(String goalName) {
      this.goalName = goalName;
      return this;
    }

    public Builder goalStatus(String goalStatus) {
      this.goalStatus = goalStatus;
      return this;
    }

    public Builder goalStartMs(long goalStartMs) {
      this.goalStartMs = goalStartMs;
      return this;
    }

    public Builder goalStartMs(Long goalStartMs) {
      this.goalStartMs = goalStartMs;
      return this;
    }

    public Builder goalEndMs(Long goalEndMs) {
      this.goalEndMs = goalEndMs;
      return this;
    }

    public Builder scheduled(boolean scheduled) {
      this.scheduled = scheduled;
      return this;
    }

    public Builder urgentGoHome(boolean urgentGoHome) {
      this.urgentGoHome = urgentGoHome;
      return this;
    }

    public AvaStatisticsEvent build() {
      return new AvaStatisticsEvent(this);
    }
  }
}
