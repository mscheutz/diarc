/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.gui.GuiAdapter;
import org.apache.commons.lang3.tuple.Pair;

import java.util.*;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A WebSocket adapter for communication between the
 * <code>BeliefComponent</code> and the frontend Belief Viewer GUI.
 * <p>
 * Implementation note: because of the high potential volume of belief changes,
 * this class does not use notifications to receive updates. Instead, it polls
 * the <code>BeliefComponent</code> at regular intervals.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see edu.tufts.hrilab.belief.BeliefComponent BeliefComponent (the
 * corresponding <code>GuiProvider</code> for this adapter)
 */
public class BeliefAdapter extends GuiAdapter {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Timer update period, in milliseconds.
   */
  public static final int UPDATE_PERIOD = 1000;

  //==========================================================================
  // Fields
  //==========================================================================
  /**
   * An array of previous beliefs, refreshed on each update. Used to find deltas
   * for asserted and retracted beliefs.
   */
  private String[] previousBeliefs = new String[]{};

  /**
   * An unformatted array of beliefs used as a buffer.
   */
  private String[] unformattedBeliefs = new String[]{};

  /**
   * A list of current beliefs.
   */
  private final List<String> currentBeliefs = new LinkedList<>();

  /**
   * A list holding the timeline of belief changes.
   */
  private final List<String> timelineBeliefs = new LinkedList<>();

  /**
   * Timer that periodically updates the client with belief info.
   */
  @SuppressWarnings("FieldCanBeLocal")
  private Timer updateTimer;

  /**
   * Lock object for message-send synchronization.
   */
  private final Object lock = new Object();

  /**
   * Tracks the current memory level that the GUI wants access to.
   */
  private MemoryLevel memoryLevel = MemoryLevel.WORKING;

  /**
   * Flag to clear the timeline when sending the next update. This is turned
   * on when the memory level is changed and turned back off when sending.
   */
  private boolean clearTimeline = false;

  /**
   * Lock object for lists.
   */
  private final ReentrantLock listLock = new ReentrantLock();

  //==========================================================================
  // Constructor
  //==========================================================================

  /**
   * Constructor.
   */
  public BeliefAdapter(Collection<String> groups) {
    super(groups);
  }

  //==========================================================================
  // Methods
  //==========================================================================

  /**
   * Get a list of all facts and rules believed. Currently, a workaround since
   * the prover's theory is not exposed as a TRADE service.
   *
   * @return concatenated list of facts and rules.
   * @throws TRADEException if the TRADE calls used to get the facts and rules
   *                        fail.
   */
  private String[] getFactsRules() throws TRADEException {
    @SuppressWarnings("unchecked assignment")
    List<Term> facts = TRADE.getAvailableService(
            new TRADEServiceConstraints()
                    .returnType(List.class)
                    .name("getFacts")
                    .argTypes(MemoryLevel.class)
    ).call(List.class, memoryLevel);

    @SuppressWarnings("unchecked assignment")
    List<Pair<Term, List<Term>>> rules = TRADE.getAvailableService(
            new TRADEServiceConstraints()
                    .returnType(List.class)
                    .name("getRules")
                    .argTypes(MemoryLevel.class)
    ).call(List.class, memoryLevel);

    String[] result = new String[facts.size() + rules.size()];
    for (int i = 0; i < result.length; i++) {
      if (i < facts.size()) {
        result[i] = facts.get(i).toString();
        continue;
      }

      Pair<Term, List<Term>> p = rules.get(i - facts.size());
      StringBuilder sb = new StringBuilder();
      sb.append(p.getLeft().toString())
              .append(" :- ");
      for (Term t : p.getRight()) {
        sb.append(t.toString())
                .append(", ");
      }
      sb.delete(sb.length() - 2, sb.length());
      result[i] = sb.toString();
    }
    return result;
  }

  /**
   * Reads unformatted beliefs from the buffer into the specified list.
   *
   * @param list list to <em>prepend</em> formatted beliefs into.
   */
  private void formatList(List<String> list) {
    listLock.lock();
    try {
      for (String s : unformattedBeliefs) {
        // Empty belief -> add blank to front of list
        if (s.isEmpty()) {
          list.add(0, "\n");
        }
        // Full belief -> make sure not just a close parenthesis
        else if (s.replace(" ", "").length() > 1) {
          list.add(0, s);
        }
      }
    } finally {
      listLock.unlock();
    }
  }

  /**
   * Given an array of beliefs, updates the timeline based on the delta
   * between this array and the currently-held belief array.
   *
   * @param beliefs incoming array of beliefs
   */
  private void updateTimelineBeliefs(String[] beliefs) {
    listLock.lock();
    try {
      List<String> toUpdate = new ArrayList<>();
      if (beliefs != previousBeliefs) {
        for (String s : beliefs) {
          if (!Arrays.asList(previousBeliefs).contains(s)) {
            toUpdate.add("assert:" + s);
          }
        }
        for (String s : previousBeliefs) {
          if (!Arrays.asList(beliefs).contains(s)) {
            toUpdate.add("retract:" + s);
          }
        }
      }

      String[] updatedList = new String[toUpdate.size()];
      for (int i = 0; i < toUpdate.size(); i++) {
        updatedList[i] = toUpdate.get(i);
      }
      unformattedBeliefs = updatedList;
      formatList(timelineBeliefs);
    } finally {
      listLock.unlock();
    }
  }

  /**
   * Given an array of beliefs, copies over the array to the list of current
   * beliefs.
   *
   * @param beliefs incoming array of beliefs.
   */
  private void updateCurrentBeliefs(String[] beliefs) {
    listLock.lock();
    try {
      currentBeliefs.clear();
      unformattedBeliefs = beliefs;
      formatList(currentBeliefs);
    } finally {
      listLock.unlock();
    }
  }

  /**
   * Updates the user about system beliefs.
   */
  public void sendBeliefs() {
    listLock.lock();
    try {
      String[] allBeliefsArray;
      try {
        allBeliefsArray = getFactsRules();
      } catch (TRADEException e) {
        log.error("Failed to get facts and rules", e);
        return;
      }

      updateTimelineBeliefs(allBeliefsArray);
      updateCurrentBeliefs(allBeliefsArray);
      previousBeliefs = allBeliefsArray;

      if (clearTimeline) {
        timelineBeliefs.clear();
      }

      Gson gson = new Gson();
      JsonObject update = new JsonObject();
      update.add("current", gson.toJsonTree(currentBeliefs));
      update.add("timeline", gson.toJsonTree(timelineBeliefs));
      update.addProperty("timelineCleared", clearTimeline);
      update.addProperty("path", getPath());

      clearTimeline = false;

      try {
        synchronized (lock) {
          sendMessage(update);
        }
      } catch (TRADEException e) {
        log.error("Failed to send message", e);
      }
    } finally {
      listLock.unlock();
    }
  }

  /**
   * Handle a user query.
   *
   * @param input query input.
   */
  private void handleQuery(String input) {
    try {
      Predicate query = Factory.createPredicate(input);
      @SuppressWarnings("unchecked assignment")
      List<Map<Variable, Symbol>> bindings = TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .returnType(List.class)
                      .name("queryBelief")
                      .argTypes(Term.class, MemoryLevel.class)
      ).call(List.class, query, memoryLevel);

      String result;
      switch (bindings.toString()) {
        case "[{}]" -> result = "true";
        case "[]" -> result = "false";
        default -> result = bindings.toString();
      }

      synchronized (lock) {
        JsonObject json = new JsonObject();
        json.addProperty("result", result);
        json.addProperty("path", getPath());
        sendMessage(json);
      }
    } catch (TRADEException e) {
      log.error("Handle query failed", e);
    }
  }

  /**
   * Handle a user assertion.
   *
   * @param input assertion input.
   */
  private void handleAssertion(String input) {
    try {
      Predicate query = Factory.createPredicate(input);
      TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .returnType(void.class)
                      .name("assertBelief")
                      .argTypes(Term.class, MemoryLevel.class)
      ).call(void.class, query, memoryLevel);
    } catch (Exception e) {
      log.error("Failure during belief assertion", e);
    }
  }

  /**
   * Handle a user retraction.
   *
   * @param input retraction input.
   */
  private void handleRetraction(String input) {
    try {
      Predicate query = Factory.createPredicate(input);
      TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .returnType(void.class)
                      .name("retractBelief")
                      .argTypes(Term.class, MemoryLevel.class)
      ).call(void.class, query, memoryLevel);
    } catch (TRADEException e) {
      log.error("Failure during belief retraction", e);
    }
  }

  /**
   * Change to the desired memory level.
   *
   * @param input string descriptor of the memory level. May be one of the
   *              following: <code>{"universal", "episodic", "working"}</code>.
   */
  private void changeMemoryLevel(String input) {
    switch (input) {
      case "universal" -> memoryLevel = MemoryLevel.UNIVERSAL;
      case "episodic" -> memoryLevel = MemoryLevel.EPISODIC;
      case "working" -> memoryLevel = MemoryLevel.WORKING;
      default -> {
        log.error("Invalid input string: \"{}\"", input);
        return;
      }
    }
    clearTimeline = true;
  }

  /**
   * Starts the update timer when connected to the client.
   */
  private void onConnect() {
    updateTimer = new Timer();
    updateTimer.scheduleAtFixedRate(new TimerTask() {
      @Override
      public void run() {
        sendBeliefs();
      }
    }, 0, UPDATE_PERIOD);
  }

  //==========================================================================
  // Implement methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  protected void init() {
    listLock.lock();
    try {
      try {
        this.previousBeliefs = getFactsRules();
        this.unformattedBeliefs = previousBeliefs;
        formatList(currentBeliefs);
      } catch (TRADEException te) {
        log.error("Failed to get facts and rules", te);
      }
    } finally {
      listLock.unlock();
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  /**
   * Handle a user message.
   *
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    String method = message.get("method").getAsString();
    String input = "";
    if (message.has("input")) {
      input = message.get("input").getAsString(); // startup message doesn't contain input
    }

    switch (method) {
      case "startup" -> onConnect();
      case "query" -> handleQuery(input);
      case "assert" -> handleAssertion(input);
      case "retract" -> handleRetraction(input);
      case "changeMemoryLevel" -> changeMemoryLevel(input);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  public String getPathRoot() {
    return "belief";
  }

}
