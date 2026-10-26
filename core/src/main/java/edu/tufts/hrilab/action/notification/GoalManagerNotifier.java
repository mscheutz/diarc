/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.notification;


import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.listener.DatabaseListener;
import edu.tufts.hrilab.action.listener.GoalListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * Notification mechanism for the GoalManagerComponent. Each NotificationType
 * has an expected callback signature:
 *
 * GOAL_STATUS_CHANGE: callBackService(GoalInfo info);
 * DATABASE_ADDITION: callBackService(ActionDBEntry dbEntry);
 * DATABASE_REMOVAL: callBackService(ActionDBEntry dbEntry);
 */
public class GoalManagerNotifier implements DatabaseListener, GoalListener {
  private static Logger log = LoggerFactory.getLogger(GoalManagerNotifier.class);

  /**
   * Maps of callback services that have requested notifications.
   */
  private final Set<TRADEServiceInfo> dbAdditionCallbacks = Collections.synchronizedSet(new HashSet<>());
  private final Set<TRADEServiceInfo> dbRemovalCallbacks = Collections.synchronizedSet(new HashSet<>());
  private final Set<TRADEServiceInfo> goalStatusCallbacks = Collections.synchronizedSet(new HashSet<>());

  public void registerNotification(TRADEServiceInfo callback, NotificationType type) {
    Set<TRADEServiceInfo> callbacks = switch (type) {
      case GOAL_STATUS_CHANGE -> goalStatusCallbacks;
      case DATABASE_ADDITION -> dbAdditionCallbacks;
      case DATABASE_REMOVAL -> dbRemovalCallbacks;
    };
    synchronized (callbacks) {
      if (callbacks.stream().anyMatch(c -> c.matches(callback))) {
        log.warn("[registerNotification] attempt to register duplicate callback, not adding");
      } else {
        callbacks.add(callback);
      }
    }
  }

  private List<TRADEServiceInfo> snapshot(Set<TRADEServiceInfo> callbacks) {
    synchronized (callbacks) {
      return new ArrayList<>(callbacks);
    }
  }

  public void unregisterNotification(TRADEServiceInfo callback, NotificationType type) {
    switch (type) {
      case GOAL_STATUS_CHANGE -> goalStatusCallbacks.remove(callback);
      case DATABASE_ADDITION -> dbAdditionCallbacks.remove(callback);
      case DATABASE_REMOVAL -> dbRemovalCallbacks.remove(callback);
    }
  }

  @Override
  public void actionAdded(ActionDBEntry adb) {
    for (TRADEServiceInfo callback : snapshot(dbAdditionCallbacks)) {
      try {
        callback.call(void.class, adb);
      } catch (TRADEException e) {
        log.error("Exception sending actionAdded notification.", e);
      }
    }
  }

  @Override
  public void actionRemoved(ActionDBEntry adb) {
    for (TRADEServiceInfo callback : snapshot(dbRemovalCallbacks)) {
      try {
        callback.call(void.class, adb);
      } catch (TRADEException e) {
        log.error("Exception sending actionRemoved notification.", e);
      }
    }
  }

  @Override
  public void goalStatusChanged(GoalInfo info) {
    List<TRADEServiceInfo> servicesToRemove = new ArrayList<>();
    for (TRADEServiceInfo callback : snapshot(goalStatusCallbacks)) {
      try {
        callback.call(void.class, new GoalInfo(info));
      } catch (TRADEException | NullPointerException e) {
        log.error("Exception sending goalStatusChanged notification.", e);
        //TODO: what is the scope of TRADEExceptions for which we want to unregister the service?
        if (e.getMessage() != null && (e.getMessage().startsWith("Service not available any longer") || e.getMessage().equals("Cannot read field \"lock\" because \"tsi\" is null"))) {
          servicesToRemove.add(callback);
        }
      }
    }
    //If a remote container has gone down without unregistering for notifications,
    // then remove the service rather than continuously log errors.
    for (TRADEServiceInfo service: servicesToRemove) {
      log.info("[notifyDialogueHistorySubscribers] unregistering unavailable service from dialogue history notifications ");
      unregisterNotification(service,NotificationType.GOAL_STATUS_CHANGE);
    }
  }
}