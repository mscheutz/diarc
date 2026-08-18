/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.listener.DatabaseListener;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.lang.annotation.Annotation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import edu.tufts.hrilab.util.resource.Resources;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


import static java.lang.String.format;

import ai.thinkingrobots.trade.*;

/**
 * @author willie
 */
public class Database {

  private final static Logger log = LoggerFactory.getLogger(Database.class);

  /**
   * Container for all available primitive and script actions (non-observer and non-recovery policy actions).
   */
  private ActionDatabase actionDB = new ActionDatabase();
  /**
   * Container for all primitive and script observers.
   */
  private ObserverDatabase observerDB = new ObserverDatabase();
  /**
   * Container for all ASL operators.
   */
  private OperatorDatabase operatorDB = new OperatorDatabase();
  /**
   * Container for all available primitive and script recovery policies.
   */
  private RecoveryPolicyDatabase recoveryPolicyDB = new RecoveryPolicyDatabase();
  /**
   * Container for all performance measures.
   */
  private PerformanceMeasuresDatabase performanceMeasuresDB = new PerformanceMeasuresDatabase();

  /**
   * Eventually, this will be used to cache POWER agentRefs to agent names (e.g., agent_3 --> bill).
   * Currently, agent names are not being resolved by POWER, so we simply keep a map of
   * names to names (e.g., bill --> bill).
   */
  private final Map<Symbol, Symbol> agents = new HashMap<>();


  /**
   * Objects that would like to be notified of changes.
   */
  private final Set<DatabaseListener> dbListeners = ConcurrentHashMap.newKeySet();

  /**
   * The action constraints, none for now, but should be set by the GM.
   */
  private TRADEServiceConstraints constraints = new TRADEServiceConstraints();

  /**
   * The currently available services with the @Action annotation.
   */
  private Set<TRADEServiceInfo> availableservices = new HashSet<>();

  /**
   * Database singleton.
   */
  private static Database instance = null;

  /**
   * Database singleton lock.
   */
  private static Lock instanceLock = new ReentrantLock();

  /**
   * Get Database singleton.
   *
   * @return
   */
  public static Database getInstance() {
    instanceLock.lock();
    try {
      if (instance == null) {
        instance = new Database();
        instance.initializeDatabase();
        try {
          TRADE.registerAllServices(instance,new ArrayList<>());
        } catch (TRADEException e) {
          log.error("Error registering action Database with TRADE.", e);
        }
      }
      return instance;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Destroy Database singleton.
   */
  public static void destroyInstance() {
    instanceLock.lock();
    try {
      if (instance != null) {
        instance.actionDB.removeScripts();

        try {
          TRADE.cancelNotification(instance, "joined",null);
          TRADE.cancelNotification(instance, "left",null);
          TRADE.deregister(instance.actionDB);
          TRADE.deregister(instance.operatorDB);
          TRADE.deregister(instance.observerDB);
          TRADE.deregister(instance.performanceMeasuresDB);
          TRADE.deregister(instance);
        } catch (TRADEException e) {
          log.error("Error de-registering action Database with TRADE.", e);
        }
        instance = null;
      }
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Convenience method to get action only DB (calls getInstance() internally).
   *
   * @return
   */
  public static ActionDatabase getActionDB() {
    instanceLock.lock();
    try {
      return getInstance().actionDB;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Convenience method to get operator only DB (calls getInstance() internally).
   *
   * @return
   */
  public static OperatorDatabase getOperatorDB() {
    instanceLock.lock();
    try {
      return getInstance().operatorDB;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Convenience method to get observer only DB (calls getInstance() internally).
   *
   * @return
   */
  public static ObserverDatabase getObserverDB() {
    instanceLock.lock();
    try {
      return getInstance().observerDB;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Convenience method to get recovery policy only DB (calls getInstance() internally).
   *
   * @return
   */
  public static RecoveryPolicyDatabase getRecoveryPolicyDB() {
    instanceLock.lock();
    try {
      return getInstance().recoveryPolicyDB;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Convenience method to get performance measures DB (calls getInstance() internally).
   *
   * @return
   */
  public static PerformanceMeasuresDatabase getPerformanceMeasuresDB() {
    instanceLock.lock();
    try {
      return getInstance().performanceMeasuresDB;
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Private constructor to enforce singleton usage.
   */
  private Database() {
    //TODO:brad should the DB ever be in any groups?
    Collection<String> groups = new HashSet<>();
    try {
      TRADE.registerAllServices(actionDB,groups);
      TRADE.registerAllServices(operatorDB,groups);
      TRADE.registerAllServices(observerDB,groups);
//      TRADE.registerAllServices(recoveryPolicyDB,groups);
      TRADE.registerAllServices(performanceMeasuresDB,groups);
    } catch (TRADEException e) {
      log.error("Error registering DB with TRADE.", e);
    }
  }

  /**
   * DB initialization. Adds default "self" agent, and registered for notifications
   * about TRADE primitive actions/observers/recovery-policies.
   */
  private void initializeDatabase() {
    // TODO: we should allow to pass in constraints here, so that actions can be constrained to groups or hosts, for example
    try {
      TRADE.requestNotification(this, "joined", constraints, null, "addPrimitiveActions");

      // initialize the data based with the currently available actions in the system
      addPrimitiveActions(null);
    } catch (TRADEException te) {
      log.error("Could not register notifications and/or obtain available services.", te);
    }
  }

  /**
   * Write all scripts in the Database to an ASL file.
   */
  protected void writeDB(String path) {
    ActionScriptLanguageWriter write = new ActionScriptLanguageWriter();
    List<ActionDBEntry> entries = new ArrayList<>();
    entries.addAll(actionDB.getScripts());
    entries.addAll(observerDB.getScripts());
    write.writeToFile(entries, path);
  }

  /**
   * Load database files and merge into existing database, assigning
   * the specified agent to the parsed actions.
   *
   * @param dbFileDir ASL files base dir
   * @param dbFiles   list of ASL filenames
   */
  public void loadDatabaseFiles(String dbFileDir, List<String> dbFiles) {
    for (String filename : dbFiles) {
      log.debug(format("Merging %s into action database", filename));

      if (filename.contains(":")) {
        // name:type,name:type,name:type:file.asl
        // get filename which is after the last ':'
        int fileNameIndex = filename.lastIndexOf(":");
        // get agents prior to last colon
        String allAgents = filename.substring(0, fileNameIndex);
        String dbFilename = filename.substring(fileNameIndex + 1);
        String[] agentStrings = allAgents.split(",");
        List<Symbol> agents = new ArrayList<>();
        for (String agent : agentStrings) {
          agents.add(Factory.createSymbol(agent));
        }
        loadDatabaseFromFile(Resources.createFilepath(dbFileDir, dbFilename), agents);
      } else {
        loadDatabaseFromFile(Resources.createFilepath(dbFileDir, filename));
      }
    }
  }

  /**
   * Load database from file and merge into existing database.
   *
   * @param dbfilename ASL filename
   */
  public void loadDatabaseFromFile(String dbfilename) {
    loadDatabaseFromFile(dbfilename, null);
  }

  /**
   * Load database from file and merge into existing database, assigning
   * the specified agents to the parsed actions.
   *
   * @param dbfilename ASL filename
   * @param agents      agent names
   */
  public void loadDatabaseFromFile(String dbfilename, List<Symbol> agents) {
    ActionScriptLanguageParser adbp = new ActionScriptLanguageParser();

    // expand agents to include typed and untyped versions
    if (agents != null) {
      List<Symbol> allAgents = new ArrayList<>();
      for (Symbol agent : agents) {
        getTypedAndUntypedDiarcAgents(agent).forEach(diarcAgent -> allAgents.add(diarcAgent));
      }
      agents = allAgents;
    }

    List<ActionDBEntry> parsedActions = adbp.parseFromFile(dbfilename, agents);

    if (parsedActions != null && !parsedActions.isEmpty()) {
      parsedActions.forEach(action -> addActionDBEntry(action));
    } else {
      log.warn("DB file " + dbfilename + " contains no actions");
    }

    if (agents != null) {
      log.info("Done processing DB file " + dbfilename + " for agents " + agents);
    } else {
      log.info("Done processing DB file " + dbfilename + " for all agents");
    }
  }

  /**
   * Load action probabilities into database
   *
   * @param filenames
   */
  public void loadPerformanceMeasuresFromFile(String fileDir, List<String> filenames) {
    for (String filename : filenames) {
      performanceMeasuresDB.loadPerformanceMeasures(fileDir, filename);
    }
  }

  public void addActionDBEntry(ActionDBEntry entry) {
    performanceMeasuresDB.createPerformanceMeasuresEntry(entry);
    if (entry.isObserver()) {
      observerDB.addObservationsToDB(entry);
    } else if (entry.isRecoveryPolicy()) {
      recoveryPolicyDB.addRecoveryPolicyToDB(entry);
    } else {
      actionDB.putAction(entry);
    }

    // notify listeners of added action
    notifyAddition(entry);
  }

  public void removeActionDBEntry(ActionDBEntry entry) {
    if (entry.isObserver()) {
      observerDB.removeObservationsFromDB(entry);
    } else if (entry.isRecoveryPolicy()) {
      log.warn("Can not currently remove recovery policies: " + entry);
      return;
    } else {
      actionDB.removeAction(entry);
    }

    // notify listeners of added action
    notifyRemoval(entry);
  }

  public void disableActionDBEntry(ActionDBEntry entry) {
    if (entry.isObserver()) {
      log.warn("Can not currently disable observers: " + entry);
      return;
    } else if (entry.isRecoveryPolicy()) {
      log.warn("Can not currently disable recovery policies: " + entry);
      return;
    } else {
      actionDB.disableAction(entry);
    }

    // notify listeners of added action
    notifyRemoval(entry);
  }

  private void addActionFromTSI(TRADEServiceInfo tsi) {
    availableservices.add(tsi);

    ActionDBEntry.Builder actionBuilder = new ActionDBEntry.Builder(tsi.serviceName);
    actionBuilder.setActionImplementer(tsi);
    // MS: are there other aspects in groups that should be added?
    for (String group : tsi.getGroups()) {
      if (group.startsWith("agent:")) {
        String agentStr = group.substring(group.indexOf(":") + 1);
        Symbol agent = Factory.createSymbol(agentStr);
        //todo: is it possible to verify that groups attached to TSIs of the form agent:xyz _do_ correspond to agents in belief?
        if (!isDiarcAgent(agent)) {
          log.error("[addActionFromTSI] adding TSI with agent group: " + group + " but no such diarcAgent defined in Belief");
        }

        // add both agentName and agentName:semanticType
        getTypedAndUntypedDiarcAgents(agent).forEach(diarcAgent -> actionBuilder.addAgent(diarcAgent));
      }
    }

    // MS: these extraction of arg names from parameter names need to be changed
    // TODO: set method args/roles from mapping to be added to @Action
    Class<?>[] serviceParameterTypes = tsi.getServiceParameterTypes();
    for (int i = 0; i < serviceParameterTypes.length; i++) {
      actionBuilder.addRole(new ActionBinding.Builder("?" + tsi.serviceParameterNames[i], serviceParameterTypes[i]).build());
    }

    // return type
    if (tsi.getServiceReturnType() != void.class) {
      actionBuilder.addRole(new ActionBinding.Builder("ret", tsi.getServiceReturnType()).setIsReturn(true).build());
    }

    // add conditions, effects and observations
    for (Annotation a : tsi.getAnnotations()) {
      if (a instanceof edu.tufts.hrilab.action.annotations.Condition) {
        addCondition((edu.tufts.hrilab.action.annotations.Condition) a, actionBuilder);
      } else if (a instanceof edu.tufts.hrilab.action.annotations.ConditionsContainer) {
        edu.tufts.hrilab.action.annotations.ConditionsContainer container = (edu.tufts.hrilab.action.annotations.ConditionsContainer) a;
        for (edu.tufts.hrilab.action.annotations.Condition c : container.value()) {
          addCondition(c, actionBuilder);
        }
      } else if (a instanceof edu.tufts.hrilab.action.annotations.Effect) {
        addEffect((edu.tufts.hrilab.action.annotations.Effect) a, actionBuilder);
      } else if (a instanceof edu.tufts.hrilab.action.annotations.EffectsContainer) {
        edu.tufts.hrilab.action.annotations.EffectsContainer container = (edu.tufts.hrilab.action.annotations.EffectsContainer) a;
        for (edu.tufts.hrilab.action.annotations.Effect e : container.value()) {
          addEffect(e, actionBuilder);
        }
      } else if (a instanceof edu.tufts.hrilab.action.annotations.Observes) {
        addObservation((edu.tufts.hrilab.action.annotations.Observes) a, actionBuilder);
      } else if (a instanceof edu.tufts.hrilab.action.annotations.OnInterrupt) {
        addInterruptService(a, actionBuilder);
      }
    }

    // finally, build new action, and add it to the DB
    actionBuilder.build(true);
  }

  /**
   * Adds a list of primitive actions in the provided constraints.
   * This should not be called directly and should instead be triggered by a TRADE joined notification.
   *
   * @param constraints the names of the services provided by the TSP that whose joined notification triggered this call
   */
  @TRADEService
  public synchronized void addPrimitiveActions(TRADEServiceInfo constraints) {
    // get services which are provided by the newly joined component that have Action/Observe annotations
    Set<TRADEServiceInfo> newServices = new HashSet<>();
    newServices.addAll(TRADE.getAvailableServices(edu.tufts.hrilab.action.annotations.Action.class, new TRADEServiceConstraints()));
    newServices.addAll(TRADE.getAvailableServices(edu.tufts.hrilab.action.annotations.Observes.class, new TRADEServiceConstraints()));

    log.debug("[addPrimitiveActions] primitives: " + newServices.size());

    // add the new services
    for (TRADEServiceInfo tsi : newServices) {
      // service is not yet known -- add it
      log.debug("[addPrimitiveActions] adding service: " + tsi.serviceString);
      if (!availableservices.contains(tsi)) {
        addActionFromTSI(tsi);
      }
    }
  }

  private static void addCondition(edu.tufts.hrilab.action.annotations.Condition annotation, ActionDBEntry.Builder builder) {
    Condition.Disjunction disjunction = new Condition.Disjunction(annotation.type());
    List<String> observables = Arrays.asList(annotation.observable());
    List<String> inferables = Arrays.asList(annotation.inferable());

    for (String c : annotation.condition()) {
      Predicate pred = Factory.createPredicate(c);
      Observable obs = Observable.get(observables.contains(c), inferables.contains(c));
      disjunction.or(pred, obs);
    }
    builder.addCondition(new Condition(disjunction));
  }

  private static void addEffect(edu.tufts.hrilab.action.annotations.Effect annotation, ActionDBEntry.Builder builder) {
    List<String> observables = Arrays.asList(annotation.observable());
    List<String> infer = Arrays.asList(annotation.infer());

    for (String e : annotation.effect()) {
      Predicate pred = Factory.createPredicate(e);
      Observable obs = Observable.get(observables.contains(e), infer.contains(e));
      builder.addEffect(new Effect(pred, annotation.type(), obs));
    }
  }

  private static void addObservation(Observes annotation, ActionDBEntry.Builder builder) {
    for (String o : annotation.value()) {
      builder.addObservation(Factory.createPredicate(o));
    }
  }

  private static void addInterruptService(Annotation a, ActionDBEntry.Builder builder) {
    if (a instanceof edu.tufts.hrilab.action.annotations.OnInterrupt) {
      edu.tufts.hrilab.action.annotations.OnInterrupt annotation = (edu.tufts.hrilab.action.annotations.OnInterrupt) a;
      if (!annotation.onCancelServiceCall().isEmpty()) {
        builder.addOnCancelEvent(generateInterruptEventSpec(annotation.onCancelServiceCall()));
      }
      if (!annotation.onSuspendServiceCall().isEmpty()) {
        builder.addOnSuspendEvent(generateInterruptEventSpec(annotation.onSuspendServiceCall()));
      }
      if (!annotation.onResumeServiceCall().isEmpty()) {
        builder.addOnResumeEvent(generateInterruptEventSpec(annotation.onResumeServiceCall()));
      }
    } else {
      log.error("Calling addInterruptService but the annotation is not an OnInterrupt.");
    }
  }

  private static EventSpec generateInterruptEventSpec(String serviceCallStr) {
    Predicate onCancelPredicate = Factory.createPredicate(serviceCallStr);
    EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.TSC);
    esBuilder.setActor("?actor");
    esBuilder.setCommand(onCancelPredicate.getName());
    esBuilder.setInputArgs(onCancelPredicate.getArgs().stream().map(arg -> arg.toString()).collect(Collectors.toList()));
    return esBuilder.build();
  }

  /**
   * Add listener
   *
   * @param l listener
   */
  public void addListener(DatabaseListener l) {
    dbListeners.add(l);
  }

  /**
   * Remove listener
   *
   * @param l listener
   */
  public void removeListener(DatabaseListener l) {
    dbListeners.remove(l);
  }

  /**
   * Notify all listener of an addition to the DB
   *
   * @param adb added action
   */
  private void notifyAddition(ActionDBEntry adb) {
    dbListeners.forEach(l -> l.actionAdded(adb));
  }

  /**
   * Notify all listeners of a removal of an action from the DB
   *
   * @param adb removed action
   */
  private void notifyRemoval(ActionDBEntry adb) {
    dbListeners.forEach(l -> l.actionRemoved(adb));
  }

  /**
   * Check if agentRef is a DIARC agent.
   *
   * @param agentRef
   * @return
   */
  @TRADEService
  @Action
  public boolean isDiarcAgent(Symbol agentRef) {
    Set<String> agentStrings = new HashSet<>();
    Set<String> untypedAgentStrings = new HashSet<>();
    try {
      agentStrings = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllDiarcAgentsForActor").argTypes(Symbol.class,Boolean.class))
              .call(Set.class, Factory.createSymbol("self", "agent"), false);
      agentStrings.forEach(s -> untypedAgentStrings.add(s.split(":")[0]));
    } catch (TRADEException e) {
      log.error("[isDiarcAgent] error making TRADE call to getAllDiarcAgentsForActor", e);
    }
    //todo: we may have to optionally strip the type from the group?
    return agentStrings.contains(agentRef.toString()) || untypedAgentStrings.contains(agentRef.toString());
  }

  /**
   * Helper method to convert agent into its typed and untyped form.
   *
   * @param agentRef
   * @return
   */
  private Set<Symbol> getTypedAndUntypedDiarcAgents(Symbol agentRef) {
    Set<String> agentStrings = new HashSet<>();
    Set<String> untypedAgentStrings = new HashSet<>();
    try {
      agentStrings = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllDiarcAgentsForActor").argTypes(Symbol.class,Boolean.class))
              .call(Set.class, agentRef, false);
      agentStrings.forEach(s -> untypedAgentStrings.add(s.split(":")[0]));
    } catch (TRADEException e) {
      log.error("[getTypedAndUntypedDiarcAgents] error making TRADE call to getAllDiarcAgentsForActor", e);
    }

    Set<Symbol> returnAgents = new HashSet<>();
    agentStrings.forEach(s -> returnAgents.add(Factory.createSymbol(s)));
    untypedAgentStrings.forEach(s -> returnAgents.add(Factory.createSymbol(s)));
    return returnAgents;
  }

  /**
   * Get all named DIARC agents known to the system
   *
   * @return current set of all DIARC agents including both embodied agents and agent "teams"
   */
  @TRADEService
  public Set<Symbol> getDiarcAgents() {
    Set<Symbol> toReturn = new HashSet<>();
    try {
      Set<String> stringResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllDiarcAgentsForActor").argTypes(Symbol.class,Boolean.class))
              .call(Set.class, Factory.createSymbol("self", "agent"), false);
      stringResults.forEach(r -> toReturn.add(Factory.createSymbol(r)));
    } catch (TRADEException e) {
      log.error("[getDiarcAgents] error making TRADE call to getAllDiarcAgentsForActor", e);
    }
    return toReturn;
  }

}
