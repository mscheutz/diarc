/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.asl.AslValues;
import edu.tufts.hrilab.action.goal.Goal;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.concurrent.*;

/** A native child interpreter handle, not an independently queued managed goal. */
public final class AslTask {
  private record Running(Goal goal, Future<ActionStatus> future, Runnable release) { }
  private final Context context;
  private final Map<String, Class<?>> resultTypes;
  private volatile Running running;

  public AslTask(Context context, Map<String, Class<?>> resultTypes) {
    this.context = context;
    this.resultTypes = java.util.Collections.unmodifiableMap(new LinkedHashMap<>(resultTypes));
  }

  public synchronized void attach(Goal goal, Future<ActionStatus> future, Runnable release) {
    if (running != null) throw new IllegalStateException("ASL task has already started");
    running = new Running(goal, future, release);
  }

  private Running running() {
    Running value = running;
    if (value == null) throw new IllegalStateException("ASL task has not started");
    return value;
  }

  public ActionStatus getStatus() {
    if (running == null) return context.getStatus();
    Running value = running();
    ActionStatus status = context.getStatus();
    return status.isTerminated() && !value.future.isDone() ? ActionStatus.PROGRESS : status;
  }

  public boolean cancel() {
    return running().goal.cancel();
  }

  public Goal getNativeGoal() {
    return running().goal;
  }

  public boolean isStarted() { return running != null; }

  public ActionStatus await() throws InterruptedException, ExecutionException {
    Running value = running();
    ActionStatus status = value.future.get();
    value.release.run();
    return status;
  }

  /** A timeout returns the current status and does not cancel the task or publish results. */
  public ActionStatus await(long milliseconds) throws InterruptedException, ExecutionException {
    if (milliseconds < 0) throw new IllegalArgumentException("ASL task timeout must be nonnegative");
    Running value = running();
    try {
      ActionStatus status = value.future.get(milliseconds, TimeUnit.MILLISECONDS);
      value.release.run();
      return status;
    } catch (TimeoutException exception) {
      return getStatus();
    }
  }

  public List<String> getResultNames() {
    return List.copyOf(resultTypes.keySet());
  }

  public Class<?> getResultType(String name) {
    Class<?> type = resultTypes.get(name);
    if (type == null) throw new IllegalArgumentException("No exported ASL task result: " + name);
    return type;
  }

  public Object getResult(String name) {
    if (getStatus() != ActionStatus.SUCCESS) {
      throw new IllegalStateException("ASL task results require successful completion");
    }
    return AslValues.convert(context.getArgumentValue(name), getResultType(name));
  }

  public <T> T getResult(String name, Class<T> type) {
    Object value = getResult(name);
    if (!type.isInstance(value)) throw new IllegalArgumentException("ASL task result type mismatch: " + name);
    return type.cast(value);
  }
}
