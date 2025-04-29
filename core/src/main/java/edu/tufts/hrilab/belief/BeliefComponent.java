/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.belief;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.belief.operators.OperatorCheck;
import edu.tufts.hrilab.belief.provers.DCEC;
import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.belief.provers.clingo.ClingoProver;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.belief.sql.SQL;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

public class BeliefComponent extends DiarcComponent implements BeliefInterface, GuiProvider {
  @Override
  public String[] getAdapterClassNames() {
    return new String[]{
            BeliefAdapter.class.getName()
    };
  }

  private static AtomicInteger nextSQLNumber = new AtomicInteger(0);
  protected List<String> agentNames;

  // notifications
  protected final Map<Term, List<NotificationRequest>> notificationRequests = new HashMap<>();
  protected final Object notificationsLock = new Object();
  /**
   * Specify prover type (prolog, asp, etc).
   */
  protected Prover.Engine proverType = Prover.Engine.PROLOG;
  /**
   * Files to initialize prover.
   */
  protected Map<MemoryLevel, List<String>> initializationFiles = new HashMap<>();
  /**
   * Default path to belief resources (configs).
   */
  private String defaultConfigPath = "config/edu/tufts/hrilab/belief";
  /**
   * SQL path.
   */
  protected String dbpath = null;
  protected Prover currProver = null;
  protected MemoryLevel currLevel = null;

  /**
   * Cached prover that uses WORKING memory level for notifications.
   */
  private Prover notificationProver;

  /**
   * SQL instance.
   */
  private SQL sql;

  private boolean prob = false;

  /**
   * Regex to remove whitespace that isn't inside quotes. Used for populating SQL directly from .pl files.
   */
  private final String whitespaceRegex = "\\s+(?=((\\\\[\\\\\"]|[^\\\\\"])*\"(\\\\[\\\\\"]|[^\\\\\"])*\")*(\\\\[\\\\\"]|[^\\\\\"])*$)";

  // Notification class, only need here because it's specific to Belief
  private class NotificationRequest {
    Term queryTerm;
    TRADEServiceInfo callback;

    NotificationRequest(Term queryTerm, TRADEServiceInfo callback) {
      this.queryTerm = queryTerm;
      this.callback = callback;
    }

    void sendAssertionNotification(List<Map<Variable, Symbol>> bindings) {
      try {
        callback.call(void.class, queryTerm, bindings);
      } catch (TRADEException e) {
        log.error("Exception trying to send assertion notification: " + queryTerm, e);
      }
    }

    void sendRetractionNotification(List<Map<Variable, Symbol>> bindings) {
      try {
        callback.call(void.class, Factory.createPredicate("not", queryTerm), bindings);
      } catch (TRADEException e) {
        log.error("Exception trying to send retraction notification: " + queryTerm, e);
      }
    }
  }

  public BeliefComponent() {
    super();
  }

  protected void init() {
    sql = new SQL(dbpath);

    // initialize sql and notification prover from files
    log.debug("initializationFiles = " + initializationFiles);
    initializeFromFiles(initializationFiles);

  }

  @Override
  protected void shutdownComponent(){
    notificationRequests.clear();
    sql.closeConnection();
  }

  @Override
  public List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("initfile").hasArgs().argName("filename").desc("load initialization file. this currently is the same as workingfile").build());
    options.add(Option.builder("workingfile").hasArgs().argName("filename").desc("load working memory initialization file").build());
    options.add(Option.builder("episodicfile").hasArgs().argName("filename").desc("load episodic memory initialization file").build());
    options.add(Option.builder("universalfile").hasArgs().argName("filename").desc("load universal memory initialization file").build());
    options.add(Option.builder("prover").hasArg().argName("type").desc("Use prover of specified type").build());
    options.add(Option.builder("disk").hasArg().argName("databasepath").desc("Use existing sql disk located at path").build());
    options.add(Option.builder("prob").desc("uses probabilistic models").build());
    return options;
  }

  @Override
  public void parseArgs(CommandLine cmdLine) {

    if (cmdLine.hasOption("initfile")) {
      //specify file of beliefs to initialize the component with
      List<String> files = Arrays.asList(cmdLine.getOptionValues("initfile"));
      MemoryLevel level = MemoryLevel.WORKING;
      if (!initializationFiles.containsKey(level)) {
        initializationFiles.put(level, new ArrayList<>());
      }
      initializationFiles.get(level).addAll(files);
    }
    if (cmdLine.hasOption("workingfile")) {
      //specify file of beliefs to initialize the component with
      List<String> files = Arrays.asList(cmdLine.getOptionValues("workingfile"));
      MemoryLevel level = MemoryLevel.WORKING;
      if (!initializationFiles.containsKey(level)) {
        initializationFiles.put(level, new ArrayList<>());
      }
      initializationFiles.get(level).addAll(files);
    }
    if (cmdLine.hasOption("episodicfile")) {
      //specify file of beliefs to initialize the component with
      List<String> files = Arrays.asList(cmdLine.getOptionValues("episodicfile"));
      MemoryLevel level = MemoryLevel.EPISODIC;
      if (!initializationFiles.containsKey(level)) {
        initializationFiles.put(level, new ArrayList<>());
      }
      initializationFiles.get(level).addAll(files);
    }
    if (cmdLine.hasOption("universalfile")) {
      //specify file of beliefs to initialize the component with
      List<String> files = Arrays.asList(cmdLine.getOptionValues("universalfile"));
      MemoryLevel level = MemoryLevel.UNIVERSAL;
      if (!initializationFiles.containsKey(level)) {
        initializationFiles.put(level, new ArrayList<>());
      }
      initializationFiles.get(level).addAll(files);
    }
    if (cmdLine.hasOption("prover")) {
      try {
        this.proverType = Prover.Engine.valueOf(cmdLine.getOptionValue("prover"));
      } catch (IllegalArgumentException e) {
        log.error("Unknown prover type " + cmdLine.getOptionValue("prover") + " . Defaulting to Prolog." + e);
        this.proverType = Prover.Engine.valueOf("PROLOG");
      }
    }
    if (cmdLine.hasOption("disk")) {
      //specify file on disk to use as a database;
      //otherwise database will be in memory and won't be saved
      this.dbpath = cmdLine.getOptionValue("disk");
    }
    if (cmdLine.hasOption("prob")) {
      prob = true;
    }
  }

  /**
   * Initialize SQL and notification prover from files.
   *
   * @param filenames
   */
  protected void initializeFromFiles(Map<MemoryLevel, List<String>> filenames) {

    if (filenames == null || filenames.isEmpty()) {
      filenames = new HashMap<>();
      log.info("No initialization filename provided. Proceeding to start this component anyways.");
      // don't return here -- need to make sure SQL tables are created below
    }

    for (MemoryLevel level : MemoryLevel.values()) {
      // add init file into to sql, by retrieving it from prover instance
      //When a duplicate belief is added, the old rule is deleted.
      //Rules are for beliefs with a premise and conclusion
      //TODO:brad:should this be a SQL prepared statement?
      String sqlStatement = "CREATE TABLE IF NOT EXISTS '" + level + "' (belief TEXT NOT NULL, startTime BIGINT, endTime BIGINT, type TEXT NOT NULL, prob REAL, source TEXT NOT NULL)";
      sql.updateSql(sqlStatement);

      if (filenames.containsKey(level)) {

        // load file contents into sql
        String toAdd = "";
        boolean multilineComment = false;
        for (String file : filenames.get(level)) {
          InputStream in = getClass().getResourceAsStream(Resources.createFilepath(defaultConfigPath, file));
          try {
            BufferedReader br = new BufferedReader(new InputStreamReader(in));
            String line;
            while ((line = br.readLine()) != null) {
              if (multilineComment) {
                // parsing multi-line comment
                if (line.endsWith("*/")) {
                  multilineComment = false;
                }
                continue;
              } else if (line.startsWith("/*") && !line.endsWith("*/")) {
                // multi-line comment
                multilineComment = true;
                if (toAdd.length() != 0) {
                  log.warn("Encountered multi-line comment. Dropping file contents: " + toAdd);
                  toAdd = "";
                }
                continue;
              } else if (line.startsWith("%") || (line.startsWith("/*") && line.endsWith("*/"))) {
                // single line comment -- ignore line
                continue;
              } else if (line.contains("%")) {
                // comment after valid code -- remove the comment part
                int commentIndex = line.indexOf("%");
                line = line.substring(0, commentIndex);
              } else if (line.contains("/*")) {
                // comment after valid code -- remove the comment part
                int commentIndex = line.indexOf("/*");
                line = line.substring(0, commentIndex);
              }

              toAdd += line.trim();
              if (toAdd.endsWith(".")) {
                toAdd = toAdd.substring(0, toAdd.length() - 1); // remove trailing "."
                toAdd = toAdd.replaceAll(whitespaceRegex, ""); //Remove all whitespace characters (space, tab, return etc.) that aren't in quotes
                log.debug("Adding line(s) to SQL: " + toAdd);
                sql.assertSQLString(toAdd, level.name(), "init");
                toAdd = "";
              }
            }
          } catch (Exception e) {
            log.error("Error parsing file: " + file, e);
          }
        }
      }
    }


    // create prover for notifications
    notificationProver = createProver(proverType);

    // create cached prover
    currLevel = MemoryLevel.WORKING;
    currProver = createProver(proverType);


    // load everything into provers
    try {
      List<String> allFiles = new ArrayList<>();
      filenames.forEach((level, files) -> allFiles.addAll(files
              .stream().map(f -> Resources.createFilepath(defaultConfigPath, f)) // convert each filename to resourcePath + filename
              .collect(Collectors.toList())));
      notificationProver.initializeFromFiles(allFiles);
      currProver.initializeFromFiles(allFiles);
    } catch (IOException e) {
      log.error("Error initializing provers.", e);
    }
  }

  @TRADEService
  @Action
  public boolean addNotifications(Predicate queryTerm, List<Symbol> callbackTerm) {
    try {
      for(Symbol s : callbackTerm) {
        TRADEServiceInfo callbackService = TRADE.getAvailableService(new TRADEServiceConstraints().name(s.getName()).argTypes(Term.class,List.class));
        TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification").argTypes(Term.class,TRADEServiceInfo.class));
        registerService.call(void.class, queryTerm, callbackService);
      }
    } catch (TRADEException e) {
      log.warn("Could not register for belief notifications. Will try again.", e);
    }
    return false;
  }

  @Override
  synchronized public void registerForNotification(Term queryTerm, TRADEServiceInfo callback) {
    log.debug("[registerForNotification] callback: " + callback + " queryTerm: " + queryTerm);

    NotificationRequest newRequest = new NotificationRequest(queryTerm, callback);
    synchronized (notificationsLock) {
      if (notificationRequests.containsKey(queryTerm)) {
        notificationRequests.get(queryTerm).add(newRequest);
      } else {
        List<NotificationRequest> newRequestList = new ArrayList<>();
        newRequestList.add(newRequest);
        notificationRequests.put(queryTerm, newRequestList);
      }
    }
  }

  @Override
  synchronized public void unregisterForNotification(Term queryTerm, TRADEServiceInfo callback) {
    log.debug("[unregisterForNotification] callback: " + callback + " queryTerm: " + queryTerm);

    synchronized (notificationsLock) {
      if (notificationRequests.containsKey(queryTerm)) {
        Iterator<NotificationRequest> requestsItr = notificationRequests.get(queryTerm).iterator();
        while (requestsItr.hasNext()) {
          NotificationRequest request = requestsItr.next();
          if (request.callback.equals(callback)) {
            requestsItr.remove();
          }
        }
      }
    }
  }

  synchronized protected void sendNotifications() {
    synchronized (notificationsLock) {
      log.debug("Notification requests: " + notificationRequests.keySet());
      for (Term queryTerm : notificationRequests.keySet()) {

        // get all previous notifications (to all components) that were made for this queryTerm
        // notificationsToRetract will be pruned as we iterate over queryTerm results so that the remaining notificationsToRetract
        // contains only notifications that were made and are no longer true (i.e., need to be retracted)
        Map<Term, NotificationRequest> notificationsToRetract = new HashMap<>();
        notificationRequests.get(queryTerm).forEach(request -> {
          Term notificationQuery = Factory.createPredicate("notified", new Symbol(String.valueOf(request.callback.hashCode())), queryTerm);
          List<Map<Variable, Symbol>> notificationBindings = notificationProver.queryBelief(notificationQuery);
          notificationQuery.copyWithNewBindings(notificationBindings).forEach(notification -> notificationsToRetract.put(notification, request));
        });

        //3 cases
        //1) binding and no notification-> assert notification, send bindings
        //2) binding and notification-> do nothing
        //3) no binding and notification-> retract notification

        List<Map<Variable, Symbol>> queryBindings = notificationProver.queryBelief(queryTerm);
        log.trace("queryTerm: " + queryTerm);
        log.trace("queryBindings: " + queryBindings);

        Map<NotificationRequest, List<Map<Variable, Symbol>>> assertionsToSend = new HashMap<>();
        for (Map<Variable, Symbol> queryBinding : queryBindings) {

          // check if each requester has been notified of this query result
          Term boundQueryTerm = queryTerm.copyWithNewBindings(queryBinding);
          for (NotificationRequest request : notificationRequests.get(queryTerm)) {
            Term notificationTerm = Factory.createPredicate("notified", new Symbol(String.valueOf(request.callback.hashCode())), boundQueryTerm);
            if (!notificationProver.querySupport(notificationTerm)) {
              // case 1: if requester has not been notified, add query results to list of results to be sent
              if (assertionsToSend.containsKey(request)) {
                assertionsToSend.get(request).add(queryBinding);
              } else {
                List<Map<Variable, Symbol>> newList = new ArrayList<>();
                newList.add(queryBinding);
                assertionsToSend.put(request, newList);
              }

              // assert notified(caller,queryTerm) (this also asserts to the notificationProver)
              assertBelief(notificationTerm, MemoryLevel.EPISODIC);
            } else {
              // case 2: requester has already been notified (also remove from notifications list)
              notificationsToRetract.remove(notificationTerm);
            }
          }
        }

        //send assertion notifications (from case 1)
        for (NotificationRequest request : assertionsToSend.keySet()) {
          log.trace("requester: " + request.callback + " queryTerm: " + queryTerm + " bindings: " + assertionsToSend);
          request.sendAssertionNotification(assertionsToSend.get(request));
        }

        // case 3: all remaining notifications must not have query match --> query is no longer true, need to retract notification
        Map<NotificationRequest, List<Map<Variable, Symbol>>> retractionsToSend = new HashMap<>();
        for (Term notificationTerm : notificationsToRetract.keySet()) {
          // retract notified(caller,queryTerm) (this also asserts to the notificationProver)
          retractBelief(notificationTerm, MemoryLevel.EPISODIC);

          // add retraction bindings to retractionsToSend so that a single notification is sent instead of one per retraction
          NotificationRequest request = notificationsToRetract.get(notificationTerm);
          Term boundQueryTerm = (Term) notificationTerm.get(1);
          Map<Variable, Symbol> retractionBindings = queryTerm.getBindings(boundQueryTerm);
          if (retractionsToSend.containsKey(request)) {
            retractionsToSend.get(request).add(retractionBindings);
          } else {
            List<Map<Variable, Symbol>> newList = new ArrayList<>();
            newList.add(retractionBindings);
            retractionsToSend.put(request, newList);
          }
        }

        // and finally, make the retraction calls to the requesters
        for (NotificationRequest request : retractionsToSend.keySet()) {
          log.trace("requester: " + request.callback + " queryTerm: " + queryTerm + " bindings: " + retractionsToSend);
          request.sendRetractionNotification(retractionsToSend.get(request));
        }

      }
    }
  }

  private Prover createProver(Prover.Engine engine) {
    // Instantiate prover
    switch (engine) {
      case PROLOG:
        return new Prolog();
      case DCEC:
        return new DCEC();
      case CLINGO:
        return new ClingoProver();
      default:
        log.error("Cannot instantiate prover " + proverType + ". Defaulting to prolog.");
        return new Prolog();
    }
  }

  private Prover loadProver(ResultSet resultSet) {
    Prover prover = loadProver(resultSet, proverType);
    return prover;
  }

  private Prover loadProver(ResultSet resultSet, Prover.Engine engine) {
    // Start new prover
    Prover prover = createProver(engine);

    try {
      while (resultSet.next()) {
        String belief = resultSet.getString(1);
        try {
          if (belief.contains(":-")) {
            prover.assertRule(belief);
          } else {
            prover.assertBelief(belief);
          }
        } catch (Exception e) {
          log.error("Error while populating prover with belief: " + belief, e);
        }
      }
    } catch (SQLException e) {
      log.error("Error loading ResultSet into prover: ", e);
    }
    return prover;
  }

  @Override
  synchronized public void clear() {
    clear(MemoryLevel.WORKING);
  }

  @Override
  synchronized public void clear(MemoryLevel memoryLevel) {
    log.debug("clearing: " + memoryLevel);
    String sqlStatement = "DELETE FROM '" + memoryLevel + "'";
    sql.updateSql(sqlStatement);

    ResultSet resultSet = sql.querySql(sql.joinConcentricLevels(currLevel, "WHERE endTime IS NULL") + " ORDER BY startTime");
    notificationProver = loadProver(resultSet); // always reset notification prover since it contains all memory levels
    if (currLevel.includesLevel(memoryLevel)) {
      currProver.setTheory(notificationProver.getTheory());
    }
  }

  @Override
  synchronized public void assertBeliefs(Set<Term> beliefs) {
    assertBeliefs(beliefs, MemoryLevel.WORKING);
  }

  @Override
  synchronized public void assertBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel) {
    assertBeliefs(beliefs, memoryLevel, true);
  }

  /**
   * Special method for use by the OperatorCheck class, so that (fluent_)equals(...) beliefs can be asserted
   * without performing an operator check and resulting in an infinite loop.
   *
   * This method should NOT be exposed as a service.
   * @param beliefs
   * @param memoryLevel
   * @param checkOperator
   */
  synchronized public void assertBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel, boolean checkOperator) {
    assertBeliefs(beliefs, memoryLevel, Factory.createPredicate("source()"), checkOperator);
  }

  synchronized public void assertBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel, Predicate source, boolean checkOperator) {
    for (Term belief : beliefs) {
      log.debug("asserting: " + belief + " in " + memoryLevel);
      if (checkOperator && OperatorCheck.isOperator(belief)) {
        OperatorCheck.assertFluent(belief, memoryLevel, this);
      } else {
        String beliefSanitized = currProver.sanitize(belief);
        sql.assertSQLString(beliefSanitized, memoryLevel.name(), source.toString());
        notificationProver.assertBelief(beliefSanitized);
        if (currLevel.includesLevel(memoryLevel)) {
          currProver.assertBelief(beliefSanitized);
        }
      }
    }
    // check and send notifications to registered components
    sendNotifications();
  }

  @Override
  synchronized public void assertBelief(Term belief) {
    assertBelief(belief, MemoryLevel.WORKING);
  }

  @Override
  synchronized public void assertBelief(Term belief, MemoryLevel memoryLevel) {
    assertBelief(belief, memoryLevel, Factory.createPredicate("context(none,none)"));
  }

  synchronized public void assertBelief(Term belief, MemoryLevel memoryLevel, Predicate source) {
    assertBelief(belief, memoryLevel, source, true);
  }

  /**
   * Special method for use by the OperatorCheck class, so that (fluent_)equals(...) beliefs can be asserted
   * without performing an operator check and resulting in an infinite loop.
   *
   * This method should NOT be exposed as a service.
   * @param belief
   * @param memoryLevel
   * @param checkOperator
   */
  synchronized public void assertBelief(Term belief, MemoryLevel memoryLevel, Predicate source, boolean checkOperator) {
    Set<Term> beliefs = new HashSet<>();
    beliefs.add(belief);
    assertBeliefs(beliefs, memoryLevel, source, checkOperator);
  }

  //todo (pete): I added the below to fix a compile issue with belief. Possible that this was in progress work that should be updated.
  /**
   * Special method for use by the OperatorCheck class, so that (fluent_)equals(...) beliefs can be asserted
   * without performing an operator check and resulting in an infinite loop.
   *
   * This method should NOT be exposed as a service.
   * @param belief
   * @param memoryLevel
   * @param checkOperator
   */
  synchronized public void assertBelief(Term belief, MemoryLevel memoryLevel, boolean checkOperator) {
    Set<Term> beliefs = new HashSet<>();
    beliefs.add(belief);
    assertBeliefs(beliefs, memoryLevel, checkOperator);
  }

  @Override
  synchronized public void retractBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel) {
    log.debug("retracting beliefs: " + beliefs);
    for (Term belief : beliefs) {
      String beliefSanitized = currProver.sanitize(belief);
      sql.retractSQLString(beliefSanitized, memoryLevel.name());
      notificationProver.retractBelief(beliefSanitized);
      if (currLevel.includesLevel(memoryLevel)) {
        currProver.retractBelief(beliefSanitized);
      }
    }

    // check and send notifications to registered components
    sendNotifications();
  }

  @Override
  synchronized public void retractBeliefs(Set<Term> beliefs) {
    retractBeliefs(beliefs, MemoryLevel.WORKING);
  }

  @Override
  synchronized public void retractBelief(Term belief, MemoryLevel memoryLevel) {
    Set<Term> beliefs = new HashSet<>();
    beliefs.add(belief);
    retractBeliefs(beliefs, memoryLevel);
  }

  @Override
  synchronized public void retractBelief(Term belief) {
    retractBelief(belief, MemoryLevel.WORKING);
  }

  @Override
  synchronized public void assertRule(Term head, List<Term> body, MemoryLevel memoryLevel) {
    // assert to sql
    StringBuilder rule = new StringBuilder(head.toString()).append(":-(");
    for (Term term : body) {
      rule.append(term.toString()).append(",");
    }
    rule.replace(rule.length() - 1, rule.length(), ")");
    sql.assertSQLString(rule.toString(), memoryLevel.name());

    // check notifications
    notificationProver.assertRule(head, body);
    if (currLevel.includesLevel(memoryLevel)) {
      currProver.assertRule(head, body);
    }
    sendNotifications();
  }

  @Override
  synchronized public void assertRule(Term head, List<Term> body) {
    assertRule(head, body, MemoryLevel.WORKING);
  }

  @Override
  synchronized public void retractRule(Term head, List<Term> body, MemoryLevel memoryLevel) {
    //TODO:brad: sanitize head and body?
    StringJoiner rule = new StringJoiner(",", head.toString() + ":-", "");
    for (Term bt : body) {
      rule.add(bt.toString());
    }

    sql.retractSQLString(rule.toString(), memoryLevel.name());
    notificationProver.retractRule(head, body);
    if (currLevel.includesLevel(memoryLevel)) {
      currProver.retractRule(head, body);
    }
  }

  @Override
  synchronized public void retractRule(Term head) {
    for(Pair<Term, List<Term>> rule : getRules()) {
      if(rule.getLeft().equals(head)) {
        retractRule(rule.getLeft(), rule.getRight());
        return;
      }
    }
  }

  @Override
  synchronized public void retractRule(Term head, List<Term> body) {
    retractRule(head, body, MemoryLevel.WORKING);
  }

  @Override
  synchronized public BeliefComponent createClone() {
    String dbName = "-disk sim_num_" + nextSQLNumber.getAndIncrement() + ".db";
    BeliefComponent newBeliefComponent = DiarcComponent.createInstance(this.getClass(), dbName, false);

    // populate new component's prover
    if (!newBeliefComponent.notificationProver.setTheory(this.notificationProver.getTheory())) {
      log.error("[createClone] Attempted to populate prover with an invalid theory.");
    }

    // populate new component's sql database

    String sqlStatement;
    for (MemoryLevel level : MemoryLevel.values()) {
      try {
        log.debug("Transferring tables...");

        sqlStatement = "CREATE TABLE IF NOT EXISTS'" + level + "' (belief TEXT NOT NULL, startTime BIGINT, endTime BIGINT, type TEXT NOT NULL, prob REAL, source TEXT NOT NULL)";
        newBeliefComponent.sql.updateSql(sqlStatement);

        //combine facts and rules sets from source
        ResultSet facts = sql.getFacts(level);
        ResultSet rules = sql.getRules(level);
        ResultSet history = sql.getHistory(level);

        while (facts.next()) {
          sqlStatement = "INSERT INTO '" + level + "' (belief,startTime,endTime,type,prob,source) VALUES ('" + facts.getString(1) + "'," + facts.getString(2) + "," + facts.getString(3) + ",'" + facts.getString(4) + "'," + facts.getString(5) + "'," + facts.getString(6) + ")";
          newBeliefComponent.sql.updateSql(sqlStatement);
        }
        while (rules.next()) {
          sqlStatement = "INSERT INTO '" + level + "' (belief,startTime,endTime,type,prob,source) VALUES ('" + rules.getString(1) + "'," + rules.getString(2) + "," + rules.getString(3) + ",'" + rules.getString(4) + "'," + rules.getString(5) + "'," + rules.getString(6) + ")";
          newBeliefComponent.sql.updateSql(sqlStatement);
        }
        while (history.next()) {
          sqlStatement = "INSERT INTO '" + level + "' (belief,startTime,endTime,type,prob,source) VALUES ('" + history.getString(1) + "'," + history.getString(2) + "," + history.getString(3) + ",'" + history.getString(4) + "'," + history.getString(5) + "'," + history.getString(6) + ")";
          newBeliefComponent.sql.updateSql(sqlStatement);
        }
      } catch (SQLException e) {
        log.error("Couldn't transfer beliefs.", e);
      }
    }

    return newBeliefComponent;
  }

  @Override
  synchronized public List<Term> getFacts() {
    return getFacts(MemoryLevel.WORKING);
  }

  @Override
  synchronized public List<Term> getFacts(MemoryLevel memoryLevel) {
    Set<Term> facts = new HashSet<>(); // HashSet to avoid duplicates

    // get all explicit facts
    ResultSet resultSet = sql.getFacts(memoryLevel);
    facts.addAll(sql.translateResultSet(resultSet));

    // get heads of all the rules, query those to get facts through inference
    List<Pair<Term, List<Term>>> rules = getRules(memoryLevel);
    rules.forEach(rule -> {
      Term head = rule.getLeft();
      List<Map<Variable, Symbol>> headResults = queryBelief(head, memoryLevel);
      facts.addAll(head.copyWithNewBindings(headResults));
    });

    return new ArrayList<>(facts);
  }

  @Override
  synchronized public List<Pair<Term, List<Term>>> getRules() {
    return getRules(MemoryLevel.WORKING);
  }

  @Override
  synchronized public List<Pair<Term, List<Term>>> getRules(MemoryLevel memoryLevel) {
    ResultSet resultSet = sql.getRules(memoryLevel);
    return sql.translateRuleSet(resultSet);
  }

  @Override
  synchronized public List<Pair<Term, Pair<Symbol, Symbol>>> getHistory() {
    return getHistory(MemoryLevel.WORKING);
  }

  @Override
  synchronized public List<Pair<Term, Pair<Symbol, Symbol>>> getHistory(MemoryLevel memoryLevel) {
    ResultSet resultSet = sql.getHistory(memoryLevel);
    List<Pair<Term, Pair<Symbol, Symbol>>> beliefSet = new ArrayList<>();

    Symbol endtime;
    try {
      while (resultSet.next()) {
        try {
          endtime = new Symbol(String.valueOf(resultSet.getLong(3)));
        } catch (SQLException e) {
          endtime = new Symbol("null");
        }
        beliefSet.add(Pair.of(Factory.createPredicate(resultSet.getString(1)), Pair.of(new Symbol(String.valueOf(resultSet.getLong(2))), endtime)));
      }
    } catch (SQLException e) {
      log.error("[getHistory] Error.", e);
    }
    return beliefSet;
  }

//  What was true in a given time period:
//  How many successful experiments were run yesterday?
  //get successes between startime/endtime
//  Was tube alpha left 5 broken at 9:00 am on Tuesday?
  //get query on(or around?) that time, return success

//  When some something happened:
//  When was the last power outage in wing beta?
  //query for power outtage in beta, return time
//  How many times have you run experiment 5?
  //query for experiment 5 action, return size

//  The source of events:
//  What did you do to restore power to wing beta?
  //query restoring power to wing beta, get source?
//  How was experiment 3 executed yesterday?
  //query exp 3, get source?

//  The effects of events:
//  What happened when you repaired bay 2?


  @TRADEService
  @Action
  synchronized public long queryLastSource(Term query) {
    String source = querySource(query).get(0).get(0);
    Predicate p = Factory.createPredicate(source);
    long contextId = Long.parseLong(p.get(0).toString());
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("makeContextSalient")).call(void.class, contextId);
    } catch (TRADEException e) {
      log.error("[queryLastSource]",e);
    }
    return contextId;
  }

  //todo: this is a convenience method to extract the time from a query. Do we need this?
  @TRADEService
  @Action
  synchronized public Symbol queryLastTime(Term query) {
    List<String> res = querySource(query).get(0);
    String source = res.get(0);
    Predicate p = Factory.createPredicate(source);
    long contextId = Long.parseLong(p.get(0).toString());
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("makeContextSalient")).call(void.class, contextId);
    } catch (TRADEException e) {
     log.error("[queryLastTime]",e);
    }
    return Factory.createSymbol(res.get(1));
  }

  @TRADEService
  @Action
  synchronized public int queryCount(Term query) {
    return querySource(query).size();
  }

  synchronized private List<List<String>> querySource(Term query) {

    //TODO:brad: sanitize query
    //Convert variables to SQL wildcards
    Set<Variable> vars = query.getVars();
    Map<Variable, Symbol> bindings = new HashMap<>();
    for(Variable var : vars) {
      bindings.put(var, Factory.createSymbol("%"));
    }
    String sqlQuery = query.copyWithNewBindings(bindings).toString();

    ResultSet resultSet = sql.querySql("SELECT source, startTime FROM 'EPISODIC' WHERE belief LIKE '"+ sqlQuery + "' ORDER BY startTime DESC");

    //todo: construct 2D array of results
    //first list is the row from the sql query
    //second list is the cols of the result: 0:sourcePred (context(ID,parentID)), 1:time
    List<List<String>> matrix = new ArrayList<>();

    try {
      List<String> row = new ArrayList<>();
      row.add(resultSet.getString(1));
      row.add(resultSet.getString(2));
      while(resultSet.next()) {
        row = new ArrayList<>();
        row.add(resultSet.getString(1));
        row.add(resultSet.getString(2));
        matrix.add(row);
      }
      return matrix;
    } catch (SQLException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  synchronized public List<Map<Variable, Symbol>> queryBelief(Term query) {
    return queryBelief(query, MemoryLevel.WORKING);
  }

  @Override
  synchronized public List<Map<Variable, Symbol>> queryBelief(Term query, MemoryLevel memoryLevel) {
    return queryBelief(query, memoryLevel, true);
  }

  synchronized public List<Map<Variable, Symbol>> queryBelief(Term query, MemoryLevel memoryLevel, boolean checkOperator) {
    String queryName = query.getName();
    if (checkOperator && OperatorCheck.isOperator(query)) {
      return OperatorCheck.query(query, memoryLevel, this);
    } else if (queryName.equals("assert") || queryName.equals("asserta") || queryName.equals("assertz")) {
      assertBelief((Term) query.get(0), memoryLevel);
      return new ArrayList<>(new HashSet<>());
    } else if (queryName.equals("retract")) {
      retractBelief((Term) query.get(0), memoryLevel);
      return new ArrayList<>(new HashSet<>());
    } else {
      if (currLevel != memoryLevel) {
        ResultSet resultSet = sql.querySql(sql.joinConcentricLevels(memoryLevel, "WHERE endTime IS NULL") + " ORDER BY startTime");
        log.debug("loading new prover" + memoryLevel);
        currProver = loadProver(resultSet);
        currLevel = memoryLevel;
      }
      return currProver.queryBelief(query);
      //todo: unsanitize query?
    }
  }

  @Override
  synchronized public List<List<Map<Variable, Symbol>>> queryBelief(List<Term> queries) {
    return queryBelief(queries, MemoryLevel.WORKING);
  }

  @Override
  synchronized public List<List<Map<Variable, Symbol>>> queryBelief(List<Term> queries, MemoryLevel memoryLevel) {
    List<List<Map<Variable, Symbol>>> responses = new ArrayList<>();
    for (Term query : queries) {
      responses.add(queryBelief(query, memoryLevel));
    }
    return responses;
  }

  @Override
  synchronized public Boolean querySupport(Term query) {
    return querySupport(query, MemoryLevel.WORKING);
  }

  @Override
  synchronized public Boolean querySupport(Term query, MemoryLevel memoryLevel) {
    return querySupport(query, memoryLevel, true);
  }

  synchronized public Boolean querySupport(Term query, MemoryLevel memoryLevel, boolean checkOperator) {
    if (checkOperator && OperatorCheck.isOperator(query)) {
      List<Map<Variable, Symbol>> results = OperatorCheck.query(query, memoryLevel, this);
      return (!results.isEmpty());
    } else {
      if (currLevel != memoryLevel) {
        ResultSet resultSet = sql.querySql(sql.joinConcentricLevels(memoryLevel, "WHERE endTime IS NULL") + " ORDER BY startTime");
        currProver = loadProver(resultSet);
        currLevel = memoryLevel;
      }

      return currProver.querySupport(query);
    }
  }

  @Override
  synchronized public Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query) {
    return querySupportWithExplanation(query, MemoryLevel.WORKING);
  }

  @Override
  synchronized public Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query, MemoryLevel memoryLevel) {
    if (currLevel != memoryLevel) {
      ResultSet resultSet = sql.querySql(sql.joinConcentricLevels(memoryLevel, "WHERE endTime IS NULL") + " ORDER BY startTime");
      currProver = loadProver(resultSet);
      currLevel = memoryLevel;
    }
    return currProver.querySupportWithExplanation(query);
  }

  @Override
  synchronized public List<Map<Variable, Symbol>> queryAtTime(Term query, Long time) {
    //facts and rules are included if they were true for the entire span of the timeframe
    String sqlStatement = sql.joinConcentricLevels(MemoryLevel.WORKING, "WHERE startTime<= " + time + " AND (endTime IS NULL OR endTime>= " + time + ")") + " ORDER BY startTime";
    return queryWithSqlFilter(query, sqlStatement);
  }

  @Override
  synchronized public List<Map<Variable, Symbol>> queryRecentFacts(Term query, int n) {
    //construct statement to issue SQL query to database
    String sqlStatement = sql.joinConcentricLevels(MemoryLevel.WORKING, "WHERE type='fact'") + " ORDER BY startTime DESC LIMIT " + n;
    return queryWithSqlFilter(query, sqlStatement);
  }

  @Override
  synchronized public List<Map<Variable, Symbol>> queryFactsInTimeframe(Term query, Long start, Long end) {
    //facts are included if they were true for the entire span of the timeframe
    String sqlStatement = sql.joinConcentricLevels(MemoryLevel.WORKING, "WHERE type='fact' AND startTime<= " + start + " AND (endTime IS NULL OR endTime>= " + end + ")") + " ORDER BY startTime";
    return queryWithSqlFilter(query, sqlStatement);
  }

  @Override
  synchronized public Map<Term,List<Pair<Long,Long>>> queryRecency(Term query) {
    currProver = loadProver(sql.getHistory(MemoryLevel.WORKING));
    List<Map<Variable,Symbol>> results = currProver.queryBelief(query);
    Map<Term,List<Pair<Long,Long>>> dict = new HashMap<>();
    for (Map<Variable,Symbol> bindings : results) {
      Term t = query.copyWithNewBindings(bindings);
      dict.put(t, sql.queryRecency(t));
    }
    return dict;
  }

  synchronized private List<Map<Variable, Symbol>> queryWithSqlFilter(Term query, String sqlFilter) {
    ResultSet resultSet = sql.querySql(sqlFilter);
    log.warn("This is using the prover with a filter.");
    currLevel = null;
    currProver = loadProver(resultSet);
    return currProver.queryBelief(query);
  }

//
//  @Override
//  synchronized public void createTopic(String tblname) {
//    try {
//      PreparedStatement stmt = sqlconn.prepareStatement("DROP TABLE IF EXISTS " + tblname);
//      stmt.executeUpdate();
//      String sql = "CREATE TABLE " + tblname + " (belief TEXT NOT NULL, startTime BIGINT, endTime BIGINT, type TEXT NOT NULL)";
//      stmt = sqlconn.prepareStatement(sql);
//      stmt.executeUpdate();
//    } catch (SQLException e) {
//      log.error("Error creating new SQL Table: ", e);
//    }
//  }
//
//  @Override
//  synchronized public void changeTopic(String topic) {
//    //check if table exists, if not, load from file
//    ResultSet resultSet = getTableResultSet(topic);
//    if (resultSet != null) {
//      loadProver(resultSet);
//    } else {
//      try {
//        // TODO: should this load nested level files??
//        prover.initializeFromFiles(initializationFiles.get(topic));
//      } catch (Exception e) {
//        log.error(e);
//      }
//      createTopic(topic);
//      String file[] = prover.getTheory().split("\r");
//      for (String line : file) {
//        if (!line.trim().isEmpty()) assertSQLString(line);
//      }
//    }
//  }
//
//  @Override
//  synchronized public void appendTopic(String topic) {
//    ResultSet resultSet = getTableResultSet(topic);
//    try {
//      while (resultSet.next()) {
//        prover.assertBelief(Factory.createPredicate(resultSet.getString(1)));
//        String sql = "INSERT INTO '" + topic + "' (belief,startTime,endTime,type) VALUES ('" + resultSet.getString(1) + "'," + resultSet.getString(2) + "," + resultSet.getString(3) + ",'" + resultSet.getString(4) + "')";
//        PreparedStatement stmt = sqlconn.prepareStatement(sql);
//        stmt.executeUpdate();
//      }
//    } catch (Exception e) {
//      log.error("Error loading ResultSet into prover: ", e);
//    }
//  }
//
//  @Override
//  synchronized public void appendTopic(String topic, String filter) {
//    ResultSet resultSet = null;
//    try {
//      //construct statement to issue SQL query to database
//      PreparedStatement stmt = sqlconn.prepareStatement("SELECT * FROM '" + topic + "' WHERE belief LIKE '%" + filter + "%'");
//      resultSet = stmt.executeQuery();
//    } catch (SQLException e) {
//      log.error("Error getting filtered table ", e);
//    }
//
//    loadProver(resultSet);
//    createTopic(topic + "WITH" + filter);
//    currTable = topic + "WITH" + filter;
//    loadTable(resultSet, topic + "WITH" + filter);
//  }

  /**
   * This exposes the notification prover to the BeliefGUI, and should not be used by anything else.
   *
   * @return
   */
  public Prover getNotificationProver() {
    return notificationProver;
  }

  /**
   * Queries belief for all of the agents corresponding to the given actor e.g. a single diarcAgent, or a collection
   * of diarcAgents which are members of a team. Queries for all valid actor names if null passed as argument.
   *
   * @param actor for which to return all descendants in the team tree (can be name or name:type)
   * @param groupFormat true if agent:name:type form desired. false if name:type desired.
   * @return a set of agent group strings which correspond to the leaves of the team hierarchy underneath the given "actor".
   */
  @TRADEService
  public Set<String> getAllDiarcAgentsForActor(Symbol actor, boolean groupFormat) {
    List<Map<Variable, Symbol>> teamMembershipResults = new ArrayList<>();
    Variable queryVar = Factory.createVariable("X");
    teamMembershipResults = queryBelief(Factory.createPredicate("memberOf", queryVar, actor));
    Set<String> possibleGroups = new HashSet<>();
    for (Map<Variable, Symbol> solution : teamMembershipResults) {
      Symbol groupSymbol = solution.get(queryVar);
      String type = groupSymbol.getType();
      if (type.isEmpty()) { //we need to extract the type information from the agent object definition
        type = queryForAgentType(groupSymbol);
      }
      String groupString = groupSymbol.getName() + ":" + type;
      if (groupFormat) {
        groupString = "agent:" + groupString;
      }
      possibleGroups.add(groupString);
    }
    return possibleGroups;
  }

  /**
   * Get a map from team names to the set of diarcAgents which make up that team.
   *
   * @return a mapping from team names to the diarcAgents within the team.
   */
  @TRADEService
  public Map<Symbol, Set<Symbol>> getAllAgentTeams() {
    List<Map<Variable, Symbol>> queryResult = new ArrayList<>();
    Variable queryVar = Factory.createVariable("X");
    Predicate teamPredicate = Factory.createPredicate("team", queryVar);
    queryResult = queryBelief(teamPredicate);
    Set<Symbol> teamNames = new HashSet<>();
    queryResult.forEach(m -> teamNames.add(m.get(queryVar)));
    Map<Symbol, Set<Symbol>> teams = new HashMap<>();
    for (Symbol teamName : teamNames) {
      Predicate queryPred = Factory.createPredicate("memberOf", queryVar, teamName);
      queryResult = queryBelief(queryPred);
      Set<Symbol> team = new HashSet<>();
      for (Map<Variable, Symbol> result : queryResult) {
        Symbol agent = result.get(queryVar);
        if (agent.getType().isEmpty()) {
          String type = queryForAgentType(agent);
          agent = Factory.createSymbol(agent.getName(), type);
        }
        team.add(agent);
      }
      teams.put(teamName, team);
    }
    return teams;
  }

  /**
   * Utility method to query belief for an `object(agentName, X)` pattern, typically asserted in
   * a prolog file. Used to get type information for a diarcAgent when a belief query fails to bind a type
   * to the agent name.
   *
   * @param agentName name of the diarcAgent
   * @return the type as a String
   */
  private String queryForAgentType(Symbol agentName) {
    String type = "";
    Variable queryVar = Factory.createVariable("X");
    List<Map<Variable, Symbol>> queryResults = queryBelief(Factory.createPredicate("object", agentName, queryVar));
    if (queryResults.size() != 1) {
      log.error("[getAllAgentTeamsForActor] incorrect number of types associated with agent: " + agentName + ". Expecting 1 binding but found: " + queryResults);
    } else {
      type = queryResults.get(0).get(queryVar).getName();
    }
    return type;
  }

}
