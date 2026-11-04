package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// its own path so an older webapp never sees it and a newer webapp can tell whether it exists
public class AvaTaughtActionsAdapter extends GuiAdapter {

  private TRADEServiceInfo getLearnedActionPredicates;
  private TRADEServiceInfo submitGoal;
  private TRADEServiceInfo joinOnGoal;
  private Map<String, Predicate> taughtActions = new HashMap<>();

  public AvaTaughtActionsAdapter(Collection<String> groups) {
    super(groups);
  }

  @Override
  protected void init() {
    try {
      getLearnedActionPredicates = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getLearnedActionPredicates").argTypes());
      submitGoal = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("submitGoal").argTypes(Predicate.class));
      joinOnGoal = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("joinOnGoal").argTypes(Long.class, Long.class));
    } catch (TRADEException e) {
      log.error("Failed to find TRADE services", e);
      throw new RuntimeException(e);
    }
    try {
      TRADEServiceInfo changed = TRADE.getAvailableService(new TRADEServiceConstraints()
              .returnType(void.class).name("taughtActionsChanged").argTypes(ActionDBEntry.class));
      TRADEServiceInfo register = TRADE.getAvailableService(new TRADEServiceConstraints()
              .returnType(void.class).name("registerGoalManagerNotification")
              .argTypes(TRADEServiceInfo.class, NotificationType.class));
      register.call(void.class, changed, NotificationType.DATABASE_ADDITION);
      register.call(void.class, changed, NotificationType.DATABASE_REMOVAL);
    } catch (TRADEException e) {
      log.error("[init] could not register for action database notifications", e);
    }
  }

  @Override
  protected boolean providesTradeServices() {
    return true;
  }

  // a task taught or forgotten by voice changes the list without a dashboard refresh
  @TRADEService
  public void taughtActionsChanged(ActionDBEntry entry) {
    sendTaughtActions();
  }

  private void sendTaughtActions() {
    try {
      List<Predicate> actions = getLearnedActionPredicates.call(List.class);
      taughtActions = new HashMap<>();
      JsonArray names = new JsonArray();
      for (Predicate action : actions) {
        taughtActions.put(action.getName(), action);
        names.add(action.getName());
      }
      JsonObject json = new JsonObject();
      json.add("taughtActions", names);
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("[sendTaughtActions] error getting taught actions", e);
    }
  }

  // submitted as the same goal the spoken "forget how to ..." produces, so the asl wrapper and
  // the llm provider's database removal notification run the way they do for speech
  private void forgetTaughtAction(String name, String actor) {
    Predicate action = taughtActions.get(name);
    if (action == null) {
      log.warn("[forgetTaughtAction] {} is not a taught action", name);
      return;
    }
    Symbol actorSymbol = Factory.createSymbol(actor);
    List<Symbol> args = new ArrayList<>();
    args.add(actorSymbol);
    args.addAll(action.getArgs());
    Predicate signature = Factory.createPredicate(action.getName(), args);
    Predicate goal = Factory.createPredicate("forgetTaughtAction", actorSymbol, signature);
    try {
      long gid = submitGoal.call(Long.class, goal);
      GoalStatus status = joinOnGoal.call(GoalStatus.class, gid, 10000L);
      log.info("[forgetTaughtAction] {} gid {} status {}", goal, gid, status);
    } catch (TRADEException e) {
      log.error("[forgetTaughtAction] error submitting {}", goal, e);
    }
  }

  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("method").getAsString()) {
      case "startup":
      case "taughtActions":
        sendTaughtActions();
        break;
      case "forgetTaughtAction":
        forgetTaughtAction(message.get("name").getAsString(), message.get("actor").getAsString());
        sendTaughtActions();
        break;
    }
  }

  @Override
  protected String getPathRoot() {
    return "avaTaughtActions";
  }
}
