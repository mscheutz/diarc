/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft;

import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.util.Util;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import ai.thinkingrobots.trade.*;

public class NovelRunComponent extends DiarcComponent {
  private volatile AtomicBoolean isGameOver = new AtomicBoolean(false);

//    private final Predicate goal = Factory.createPredicate("goal(self, fluent_geq(world(safe), 1))");
//    private final Predicate goal = Factory.createPredicate("goal(self, fluent_geq(inventory(self, log), 10))");
  //  private final Predicate goal = Factory.createPredicate("goal(self, fluent_geq(inventory(self, rubber), 1))");
  //  private final Predicate goal = Factory.createPredicate("goal(self, fluent_geq(inventory(self, key), 1))");
//  private final Predicate goal = Factory.createPredicate("goal(self, facing_obj(self, log, two))");
//  private final Predicate goal = Factory.createPredicate("explore_rooms(self)");
  private final Predicate goal = Factory.createPredicate("goal(self, fluent_geq(inventory(self, pogo_stick), 1))"); // DO NOT DELETE
  private volatile AtomicLong goalID = new AtomicLong(-1L);

  private long startTime;
  private int trialCount = -1;
  private long maxExploreTime = 60000; // max exploration time allowed in each game (ms)
  private Symbol actor = Factory.createSymbol("self");

  public NovelRunComponent() {
    super();
  }

  @Override
  protected void init() {
    // start the main tournament loop
    new Thread(() -> playGame()).start();
  }

  @TRADEService
  public Boolean playGame() {
    // wait for submitGoal to be available in the system
    log.info("Waiting for TRADE Service for submitGoal to be available ...");
    TRADEServiceInfo tsi = null;
    while (tsi == null) {
      try {
        tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal"));
      } catch (TRADEException e) {
        Util.Sleep(100);
      }
    }

    log.info("TRADE Service for submitGoal found. Starting game loop.");

    while (true) {
      // (re)set local game info
      resetGameInfo();

      log.info("Running Trial #: " + trialCount);

      // Initializing beliefs, reporting and exploring pre-goal-execution novelties
      try {
        log.info("Initializing world.");
        tsi = null;
        while (tsi == null) {
          try {
            tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("initializeBeliefs"));
          } catch (TRADEException e) {
            Util.Sleep(1000);
          }
        }

        // TODO: remove these once PAL init bug is fixed
        TRADE.getAvailableService(new TRADEServiceConstraints().name("senseRecipe")).call(Object.class);
        TRADE.getAvailableService(new TRADEServiceConstraints().name("senseRecipe")).call(Object.class);

        // initialize belief from new game, and detect pre-game novelties
        Set<Term> novelties = tsi.call(Set.class);

        // report detected pre-game novelties (will handle them below, after interacting with traders)
        if (!novelties.isEmpty()) {
          log.info("Detected pre-execution novelties: " + novelties);
          // report novelty
          TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, novelties);

          // assert recipe and object novelties into universal so they aren't detected as novel every subsequent game
          TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, novelties, MemoryLevel.UNIVERSAL);
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////// TESTING FAILURE CASES //////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        //Predicate find_safe_goal = Factory.createPredicate("sti(self,fluent_geq(world, safe, 1))");
        //doGoal(find_safe_goal);

        //Predicate approach_actor_goal = Factory.createPredicate("facing_obj(self, trader_104, two)");
        //doGoal(approach_actor_goal);

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////// DONE TESTING FAILURE CASES /////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////


        // handle detected pre-game novelties
        if (!novelties.isEmpty()) {
          log.info("Handling pre-execution novelties: " + novelties);
          for (Term novelty : novelties) {
            // initialize other info needed in belief for various novelties (e.g., object types)
            TRADE.getAvailableService(new TRADEServiceConstraints().name("initializeNovelty")).call(Object.class, novelty);

            // generate list of explorations to try
            Set<Predicate> explorationsToTry = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateExplorationsToTry")).call(Set.class, novelty);

            // assert explorations to try, which will be removed as they're explored
            // this allows explorations to continue into subsequent games if explorations can't be completed
            TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, explorationsToTry, MemoryLevel.UNIVERSAL);
          }
        }

        // execute any remaining novelty explorations left to try
        Predicate explorationPredicate = Factory.createPredicate("toExplore(X,Y)");
        List<Map<Variable, Symbol>> explorationResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, explorationPredicate);
        List<Term> explorationsToTry = new ArrayList<>(explorationPredicate.copyWithNewBindings(explorationResults));
        log.info("Exploring novelties: " + explorationsToTry);
        long explorationStartTime = System.currentTimeMillis();
        for (Term explorationToTry : explorationsToTry) {
          if (System.currentTimeMillis() - explorationStartTime > maxExploreTime) {
            log.info("Max exploration time reached. Moving on to main goal.\n\n\n\n");
            break;
          }
          TRADE.getAvailableService(new TRADEServiceConstraints().name("executeExploration")).call(Object.class, explorationToTry);
        }

      } catch (TRADEException e) {
        log.error("Error in initialization", e);

        // Execute primary top-level goal
        boolean goalSuccess = doGoal(goal);

        // if didn't succeed and game isn't over, report novelty and give up
        log.info("Goal success: " + goalSuccess);
        if (!goalSuccess && !isGameOver.get()) {
          try {
            Set<Term> novelties = new HashSet<>();
            novelties.add(Factory.createPredicate("unknown(failure)"));
            TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, novelties);
            log.info("Giving up. Trial #" + trialCount);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("giveUp")).call(Object.class);
          } catch (TRADEException e2) {
            log.error("Error trying to give up.", e2);
          }
        }

        // wait until game is over before starting next game
        while (!isGameOver.get()) {
          log.info("Waiting for game to be over...");
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("nop")).call(Object.class);
          } catch (TRADEException e2) {
            log.error("Error waiting for game to be over.");
          }
          Util.Sleep(1000);
        }
        log.info("Game over.\n\n\n\n\n\n\n\n\n\n");
      }
    }
  }

  /**
   * Helper method to submit goal and wait for it to finish. Returns true if the goal finishes with GoalStatus SUCCEEDED,
   * and false otherwise.
   *
   * @param goal
   * @return
   */
  private boolean doGoal(Predicate goal) {
    if (isGameOver.get()) {
      log.info("Not accepting new goals until new game starts: " + goal);
      return false;
    }

    GoalStatus goalStatus = null;
    try {
      boolean known = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(Boolean.class, goal);
      if (!known) {
        log.info("Submitting goal: " + goal);
        goalID.set(TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal")).call(Long.class, goal));
        goalStatus = TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal")).call(GoalStatus.class, goalID.get());
        log.info("Goal terminated (" + goal + ") with status: " + goalStatus);
      } else {
        log.info("Goal already achieved: " + goal);
      }
    } catch (TRADEException e) {
      log.error("[doGoal] Error for goal: " + goal, e);
      return false;
    }

    if (goalStatus != null && goalStatus == GoalStatus.SUCCEEDED) {
      return true;
    } else {
      return false;
    }
  }

  private void cancelCurrentGoals() {
    try {
      List<Predicate> currGoals = TRADE.getAvailableService(new TRADEServiceConstraints().name("getCurrentGoals")).call(List.class, actor);
      for (Predicate goal : currGoals) {
        log.info("Terminating current goal: " + goal);
        Predicate cancelGoal = Factory.createPredicate("cancelGoal", actor, goal);
        long cancelGoalId = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal")).call(Long.class, cancelGoal);
        TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal")).call(Object.class, cancelGoalId);
        log.info("Goal Terminated: " + goal);
      }
    } catch (TRADEException e) {
      log.error("Error canceling goal: " + goal, e);
    }
  }

  private void resetGameInfo() {
    isGameOver.set(false);
    ++trialCount;
    startTime = System.currentTimeMillis();
  }

  @TRADEService
  public void setGameOver(boolean goalAchieved) {
    long trialTime = getTrialTime() / 1000;
    log.info("TRIAL " + trialCount + " COMPLETED in " + trialTime + " sec.");

    isGameOver.set(true);
    if (!goalAchieved) {
      cancelCurrentGoals();
    }
  }

  @TRADEService
  public long getTrialTime() {
    return System.currentTimeMillis() - startTime;
  }

  @TRADEService
  public Boolean receivedNoveltySignal() {
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(Boolean.class, Factory.createPredicate("noveltySignal", "true"));
    } catch (TRADEException e) {
      log.error("Error checking belief for novelty signal.", e);
    }
    return false;
  }

  /**
   * Check if any novelties have been reported, by checking belief for novelty(X,Y) predicates.
   *
   * @return
   */
  private boolean reportedNovelty() {
    try {
      Predicate noveltyQuery = Factory.createPredicate("novelty", "X", "Y");
      List<Map<Variable, Symbol>> reportedNovelties = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, noveltyQuery);
      if (reportedNovelties.size() > 0) {
        return true;
      }
    } catch (TRADEException e) {
      log.error("Error checking if novelty has been reported.", e);
    }

    return false;
  }

}
