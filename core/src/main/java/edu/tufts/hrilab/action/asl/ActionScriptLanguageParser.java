/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * @author Zhouqi
 */

package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.ActionDBEntry.Builder;
import edu.tufts.hrilab.action.recovery.PolicyConstraints;
import edu.tufts.hrilab.action.recovery.PostRecoveryExecutionType;
import edu.tufts.hrilab.action.recovery.RecoveryType;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.ErrorNode;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.InputStream;
import java.util.*;

public class ActionScriptLanguageParser {

  private final static Logger log = LoggerFactory.getLogger(ActionScriptLanguageParser.class);

  /**
   * Database file to parse.
   */
  private String dbFile;

  /**
   * Agents that can execute the actions in the dbfile.
   */
  private List<Symbol> agents;

  /**
   * Parse contents of dbFile into ActionDBEntry(s).
   *
   * @return parsed actions, or null if the script cannot be parsed
   */
  public List<ActionDBEntry> parseFromFile(String dbFile, List<Symbol> agents) {
    log.debug("Parsing action db file: " + dbFile + " for agents: " + agents);
    this.dbFile = dbFile;
    this.agents = agents;

    try (InputStream stream = getClass().getResourceAsStream(dbFile)) {
      if (stream == null) {
        throw new IllegalArgumentException("ASL resource not found: " + dbFile);
      }
      return parse(CharStreams.fromStream(stream));
    } catch (Exception e) {
      log.error("Error parsing file: " + dbFile, e);
      return null;
    }
  }

  /**
   * Parse action scripts from a string (i.e., asl file contents in string form).
   *
   * @param dbFile - filename to associate with ActionDBEntry
   * @param agents  - agents (possibly null)
   * @param asl    - ASL contents to parse in String form
   * @return parsed actions, or null if the script cannot be parsed
   */
  public List<ActionDBEntry> parseFromString(String dbFile, List<Symbol> agents, String asl) {
    log.debug("[parseFromString] Attempting to create ActionDBEntrys from asl: " + asl);
    this.dbFile = dbFile;
    this.agents = agents;

    try {
      return parse(CharStreams.fromString(asl));
    } catch (Exception e) {
      log.error("Error parsing file: " + dbFile, e);
      return null;
    }

  }

  /** Unlike the legacy nullable API, malformed source is reported to the caller. */
  public List<ActionDBEntry> parseStrict(String source, List<Symbol> agents, String asl) {
    this.dbFile = Objects.requireNonNull(source);
    this.agents = agents;
    try {
      return parse(CharStreams.fromString(asl));
    } catch (StackOverflowError error) {
      throw new IllegalArgumentException("ASL syntax is too deeply nested in " + source, error);
    }
  }

  private List<ActionDBEntry> parse(CharStream input) {
    List<String> errors = new ArrayList<>();
    BaseErrorListener errorListener = new BaseErrorListener() {
      @Override
      public void syntaxError(Recognizer<?, ?> recognizer, Object offendingSymbol, int line,
                              int charPositionInLine, String message, RecognitionException exception) {
        errors.add(line + ":" + charPositionInLine + " " + message);
      }
    };

    ASLLexer lexer = new ASLLexer(input);
    lexer.removeErrorListeners();
    lexer.addErrorListener(errorListener);
    ASLParser parser = new ASLParser(new CommonTokenStream(lexer));
    parser.removeErrorListeners();
    parser.addErrorListener(errorListener);
    ParseTree tree = parser.actionScript();
    if (!errors.isEmpty()) {
      throw new IllegalArgumentException("Invalid ASL: " + String.join("; ", errors));
    }

    AsEmitter converter = new AsEmitter(input.toString());
    ParseTreeWalker.DEFAULT.walk(converter, tree);
    log.debug("Successfully parsed {} !", dbFile);
    return converter.getParsedAction();
  }

  private class AsEmitter extends ASLBaseListener {
    private ParserRuleContext activeRule;
    private final NavigableMap<Integer, String> origins = new TreeMap<>();

    AsEmitter(String source) {
      String[] lines = source.split("\\R", -1);
      for (int i = 0; i < lines.length; i++) {
        String line = lines[i].trim();
        String marker = "// @diarc-origin ";
        if (line.startsWith(marker)) {
          origins.put(i + 1, new String(Base64.getDecoder().decode(line.substring(marker.length())),
              java.nio.charset.StandardCharsets.UTF_8));
        }
      }
    }

    @Override public void enterEveryRule(ParserRuleContext ctx) { activeRule = ctx; }
    @Override public void exitEveryRule(ParserRuleContext ctx) { activeRule = ctx.getParent(); }

    private SourceLocation location() {
      var origin = origins.floorEntry(activeRule.getStart().getLine());
      return new SourceLocation(dbFile, activeRule.getStart().getLine(),
          activeRule.getStart().getCharPositionInLine() + 1, origin == null ? null : origin.getValue());
    }

    private String expressionText(ParserRuleContext ctx) {
      return ctx.getStart().getInputStream().getText(org.antlr.v4.runtime.misc.Interval.of(
          ctx.getStart().getStartIndex(), ctx.getStop().getStopIndex()));
    }

    private Builder curr_dbe_builder;
    private List<ActionDBEntry> parsed_actions;
    private int controlContentDepth = 0;
    private record ParsedVar(String name, String semanticType) {}

    private Map<String, String> aliasingMap;
    private Condition.Disjunction orCondition = null;

    List<ActionDBEntry> getParsedAction() {
      return parsed_actions;
    }

    @Override
    public void enterActionScript(ASLParser.ActionScriptContext ctx) {
      aliasingMap = new HashMap<>();
      installDefaultTypeAliases();
      parsed_actions = new ArrayList<>();
      controlContentDepth = 0;
    }

    private void installDefaultTypeAliases() {
      aliasingMap.put("Predicate", "edu.tufts.hrilab.fol.Predicate");
      aliasingMap.put("Symbol", "edu.tufts.hrilab.fol.Symbol");
      aliasingMap.put("Variable", "edu.tufts.hrilab.fol.Variable");
      aliasingMap.put("Term", "edu.tufts.hrilab.fol.Term");
      aliasingMap.put("List", "java.util.List");
      aliasingMap.put("Map", "java.util.Map");
      aliasingMap.put("Set", "java.util.Set");
    }

    @Override
    public void enterImports(ASLParser.ImportsContext ctx) {
      String toName;
      String fromName = ctx.javaType().getText();
      if (ctx.alias() == null) {
        toName = fromName.substring(fromName.lastIndexOf('.') + 1);
      } else {
        toName = ctx.alias().javaType().getText();
      }
      aliasingMap.put(toName, fromName);
    }

    @Override
    public void enterDefinition(ASLParser.DefinitionContext ctx) {
      String methodName = ctx.NAME() != null ? ctx.NAME().getText() : ctx.INVOKE().getText();
      curr_dbe_builder = new Builder(methodName) {
        @Override public Builder addEventSpec(EventSpec event) {
          return super.addEventSpec(event.withSourceLocation(location()));
        }
      };
      curr_dbe_builder.setDBFile(dbFile);
      curr_dbe_builder.setSourceLocation(location());
      if (agents != null) {
        for (Symbol agent:agents) {
          curr_dbe_builder.addAgent(agent);
        }
      }
      if (ctx.inputParameter().parameterList().parameter() != null) {
        ASLParser.ParameterContext pm = ctx.inputParameter().parameterList().parameter();
        addSignatureRole(pm, false);
        ctx.inputParameter().parameterList().parameterTail().forEach(tail -> addSignatureRole(tail.parameter(), false));
      }
      if (ctx.outputParameter().parameterList().parameter() != null) {
        ASLParser.ParameterContext pm = ctx.outputParameter().parameterList().parameter();
        addSignatureRole(pm, true);
        ctx.outputParameter().parameterList().parameterTail().forEach(tail -> addSignatureRole(tail.parameter(), true));
      }
    }

    @Override
    public void exitDefinition(ASLParser.DefinitionContext ctx) {
      ActionDBEntry entry = curr_dbe_builder.build(false);
      List<String> predeclaredLocals = entry.getRoles().stream()
          .filter(ActionBinding::isLocal)
          .map(ActionBinding::getName)
          .collect(java.util.stream.Collectors.toList());
      AslScopeValidator.validate(entry.getEventSpecs(), predeclaredLocals);
      parsed_actions.add(entry);
    }

    @Override
    public void enterDescription(ASLParser.DescriptionContext ctx) {
      String desc = stripQuotes(ctx.SENTENCE().getText());
      curr_dbe_builder.setDescription(desc);
    }

    @Override
    public void enterLocalvar(ASLParser.LocalvarContext ctx) {
      String roleName = ctx.LOCALVAR() != null ? ctx.LOCALVAR().getText() : ctx.GLOBALVAR().getText();
      ParsedVar parsedVar = parseVar(roleName);
      if (controlContentDepth == 0) {
        addRole(roleName, ctx.javaType().getText(), ctx.assignment(), true, false);
      } else {
        emitBlockLocalDeclaration(parsedVar.name(), parsedVar.semanticType(), ctx.javaType().getText(), ctx.assignment());
      }
    }

    @Override
    public void enterControlContent(ASLParser.ControlContentContext ctx) {
      controlContentDepth++;
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL);
      ParserRuleContext r = ctx.getParent();
      if (r instanceof ASLParser.IfStatementContext || r instanceof ASLParser.ElifStatementContext) {
        esBuilder.setCommand("then");
      } else if (r instanceof ASLParser.WhileStatementContext) {
        esBuilder.setCommand("do");
      } else if (r instanceof ASLParser.BlockStatementContext) {
        esBuilder.setCommand("block");
      } else if (r instanceof ASLParser.ForStatementContext || r instanceof ASLParser.ForEachStatementContext) {
        esBuilder.setCommand("block");
      } else {
        return;
      }

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitControlContent(ASLParser.ControlContentContext ctx) {
      ParserRuleContext r = ctx.getParent();
      if (r instanceof ASLParser.ForStatementContext
          || r instanceof ASLParser.ForEachStatementContext
          || r instanceof ASLParser.BlockStatementContext) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endblock");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
      controlContentDepth = Math.max(0, controlContentDepth - 1);
    }

    @Override
    public void enterSetting(ASLParser.SettingContext ctx) {
      String attribute = ctx.NAME().getText();
      String value = ctx.numberLiteral().getText();
      switch (attribute) {
        case "benefit":
          curr_dbe_builder.setBenefit(value);
          break;
        case "cost":
          curr_dbe_builder.setCost(value);
          break;
        case "minurg":
          curr_dbe_builder.setMinUrg(value);
          break;
        case "maxurg":
          curr_dbe_builder.setMaxUrg(value);
          break;
        case "timeout":
          curr_dbe_builder.setTimeout(value);
          break;
        default:
          break;
      }
    }

    @Override
    public void enterOrCondition(ASLParser.OrConditionContext ctx) {
      if (!ctx.preCondition().isEmpty()) {
        orCondition = new Condition.Disjunction(ConditionType.PRE);
      } else if (!ctx.overallCondition().isEmpty()) {
        orCondition = new Condition.Disjunction(ConditionType.OVERALL);
      } else {
        orCondition = new Condition.Disjunction(ConditionType.OBLIGATION);
      }
    }

    @Override
    public void exitOrCondition(ASLParser.OrConditionContext ctx) {
      curr_dbe_builder.addCondition(new Condition(orCondition));
      orCondition = null;
    }

    @Override
    public void enterPreCondition(ASLParser.PreConditionContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      if (orCondition == null) {
        curr_dbe_builder.addCondition(new Condition(predicate, ConditionType.PRE, obs));
      } else {
        orCondition.or(predicate, obs);
      }
    }

    @Override
    public void enterOverallCondition(ASLParser.OverallConditionContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      if (orCondition == null) {
        curr_dbe_builder.addCondition(new Condition(predicate, ConditionType.OVERALL, obs));
      } else {
        orCondition.or(predicate, obs);
      }
    }

    @Override
    public void enterObligationCondition(ASLParser.ObligationConditionContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      if (orCondition == null) {
        curr_dbe_builder.addCondition(new Condition(predicate, ConditionType.OBLIGATION, obs));
      } else {
        orCondition.or(predicate, obs);
      }
    }

    @Override
    public void enterAlwaysEffect(ASLParser.AlwaysEffectContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      curr_dbe_builder.addEffect(new Effect(predicate, EffectType.ALWAYS, obs));
    }

    @Override
    public void enterSuccessEffect(ASLParser.SuccessEffectContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      curr_dbe_builder.addEffect(new Effect(predicate, EffectType.SUCCESS, obs));
    }

    @Override
    public void enterFailureEffect(ASLParser.FailureEffectContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      curr_dbe_builder.addEffect(new Effect(predicate, EffectType.FAILURE, obs));
    }

    @Override
    public void enterNonperfEffect(ASLParser.NonperfEffectContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      Observable obs;
      if (ctx.OBSERVATION() != null) {
        obs = Observable.TRUE;
      } else if (ctx.INFER() != null) {
        obs = Observable.FALSE;
      } else {
        obs = Observable.DEFAULT;
      }
      curr_dbe_builder.addEffect(new Effect(predicate, EffectType.NONPERF, obs));
    }

    private Class<?> getClassFromString(String className) {
      String typeName = aliasingMap.containsKey(className) ? aliasingMap.get(className) : className;
      Class<?> type = Utilities.getClass(typeName);
      if (type == null) {
        log.error("[getClassFromString] Class not found found for type: " + typeName + ". will use default to String representation.");
        type = String.class;
      }
      return type;
    }

    private EventSpec generateOnInterruptEventSpec(ASLParser.InterruptSpecContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(getEventType(ctx.interruptSpecType()));

      // get actor (optional)
      if (ctx.interruptSpecType().name() != null) {
        esBuilder.setActor(ctx.interruptSpecType().name().getText());
      }

      // get function name
      esBuilder.setCommand(ctx.function().name().getText());

      // get input args
      esBuilder.setInputArgs(getInputArgs(ctx.function().inputList()));

      return esBuilder.build();
    }

    @Override
    public void enterOnCancel(ASLParser.OnCancelContext ctx) {
      if (curr_dbe_builder.hasOnCancelEvent()) {
        log.error("More than one CANCEL event defined. Ignoring: {}", ctx.getText());
        return;
      }
      curr_dbe_builder.addOnCancelEvent(generateOnInterruptEventSpec(ctx.interruptSpec()));
    }

    @Override
    public void enterOnSuspend(ASLParser.OnSuspendContext ctx) {
      if (curr_dbe_builder.hasOnSuspendEvent()) {
        log.error("More than one SUSPEND event defined. Ignoring: {}", ctx.getText());
        return;
      }
      curr_dbe_builder.addOnSuspendEvent(generateOnInterruptEventSpec(ctx.interruptSpec()));
    }

    @Override
    public void enterOnResume(ASLParser.OnResumeContext ctx) {
      if (curr_dbe_builder.hasOnResumeEvent()) {
        log.error("More than one RESUME event defined. Ignoring: {}", ctx.getText());
        return;
      }
      if (ctx.interruptSpec() != null) {
        curr_dbe_builder.addOnResumeEvent(generateOnInterruptEventSpec(ctx.interruptSpec()));
      } else if (ctx.resumeResetSpec() != null) {
        EventSpec.Builder eventSpec = new EventSpec.Builder(EventSpec.EventType.CONTROL);
        eventSpec.setCommand("reset");
        curr_dbe_builder.addOnResumeEvent(eventSpec.build());
      }
    }

    @Override
    public void enterObserves(ASLParser.ObservesContext ctx) {
      Predicate predicate = processPredicate(ctx.predicate());
      curr_dbe_builder.addObservation(predicate);
    }

    @Override
    public void enterRecovery(ASLParser.RecoveryContext ctx) {
      PolicyConstraints policyConstraints = new PolicyConstraints();
      if (!ctx.actionRecoveryDetails().isEmpty()) {
        if (ctx.actionRecoveryDetails().size() > 1) {
          log.warn("[enterRecovery] Multiple actionRecovery fields detected, only using first");
        }
        policyConstraints.recoveryType = RecoveryType.ACTION;
        if (ctx.actionRecoveryDetails(0).actionRecoveryCatchingActions() != null) {
          ctx.actionRecoveryDetails(0).actionRecoveryCatchingActions().predicateList().predicate().forEach(pred -> policyConstraints.catchingActions.add(processPredicate(pred)));
        }
        policyConstraints.postRecoveryExecutionType = PostRecoveryExecutionType.fromString(ctx.actionRecoveryDetails(0).postActionRecoveryBehavior().postActionRecoveryType().getText());
      }

      if (!ctx.recoveryGoals().isEmpty()) {
        if (ctx.recoveryGoals().size() > 1) {
          log.warn("[enterRecovery] Multiple goals fields detected, only using first");
        }
        ctx.recoveryGoals(0).predicateList().predicate().forEach(pred -> policyConstraints.goals.add(processPredicate(pred)));
      }
      if (!ctx.recoveryExcludedGoals().isEmpty()) {
        if (ctx.recoveryGoals(0) != null) {
          log.warn("[enterRecovery] goals and excludedGoals cannot both be defined, ignoring supplied excludedGoals");
        } else {
          if (ctx.recoveryExcludedGoals().size() > 1) {
            log.warn("[enterRecovery] Multiple excludedGoals fields detected, only using first");
          }
          ctx.recoveryExcludedGoals(0).predicateList().predicate().forEach(pred -> policyConstraints.excludedGoals.add(processPredicate(pred)));
        }
      }
      if (!ctx.recoveryFailedActions().isEmpty()) {
        if (ctx.recoveryFailedActions().size() > 1) {
          log.warn("[enterRecovery] Multiple failedActions fields detected, only using first");
        }
        ctx.recoveryFailedActions(0).predicateList().predicate().forEach(pred -> policyConstraints.failedActions.add(processPredicate(pred)));
      }
      if (!ctx.recoveryExcludedFailedActions().isEmpty()) {
        if (ctx.recoveryFailedActions(0) != null) {
          log.warn("[enterRecovery] failedActions and excludedFailedActions cannot both be defined, ignoring supplied excludedFailedActions");
        } else {
          if (ctx.recoveryExcludedFailedActions().size() > 1) {
            log.warn("[enterRecovery] Multiple excludedFailedActions fields detected, only using first");
          }
          ctx.recoveryExcludedFailedActions(0).predicateList().predicate().forEach(pred -> policyConstraints.excludedFailedActions.add(processPredicate(pred)));
        }
      }
      if (!ctx.recoveryFailureReasons().isEmpty()) {
        if (ctx.recoveryFailureReasons().size() > 1) {
          log.warn("[enterRecovery] Multiple failureReasons fields detected, only using first");
        }
        ctx.recoveryFailureReasons(0).predicateList().predicate().forEach(pred -> policyConstraints.failureReasons.add(processPredicate(pred)));
      }
      if (ctx.recoveryExcludedFailureReasons(0) != null) {
        if (ctx.recoveryFailureReasons(0) != null) {
          log.warn("[enterRecovery] failureReasons and excludedFailureReasons cannot both be defined, ignoring supplied excludedFailureReasons");
        } else {
          if (ctx.recoveryExcludedFailureReasons().size() > 1) {
            log.warn("[enterRecovery] Multiple excludedFailureReasons fields detected, only using first");
          }
          ctx.recoveryExcludedFailureReasons(0).predicateList().predicate().forEach(pred -> policyConstraints.excludedFailureReasons.add(processPredicate(pred)));
        }
      }
      if (!ctx.recoveryActionStatuses().isEmpty()) {
        if (ctx.recoveryActionStatuses().size() > 1) {
          log.warn("[enterRecovery] Multiple actionStatuses fields detected, only using first");
        }
        ctx.recoveryActionStatuses(0).NAME().forEach(status -> policyConstraints.actionStatuses.add(ActionStatus.fromString(status.getText())));
      }
      if (!ctx.recoveryMaximumAttempts().isEmpty()) {
        if (ctx.recoveryMaximumAttempts().size() > 1) {
          log.warn("[enterRecovery] Multiple maximumAttempts fields detected, only using first");
        }
        String numberText = ctx.recoveryMaximumAttempts(0).numberLiteral().getText();
        int number = -1;
        try {
          number = Integer.parseInt(numberText);
        } catch (NumberFormatException e) {
          log.error("[enterRecovery] maximumAttempts field must be a positive integer", e);
        }
        if (number < 1) {
          log.warn("[enterRecovery] invalid maximumAttempts value encountered {}, not using field", number);
        } else {
          policyConstraints.maximumAttempts = number;
        }
      }

      curr_dbe_builder.addRecoveryPolicyConstraint(policyConstraints);
    }

    @Override
    public void enterSpecFunction(ASLParser.SpecFunctionContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(getEventType(ctx.specType()));

      // get actor (optional)
      if (ctx.specType().name() != null) {
        esBuilder.setActor(ctx.specType().name().getText());
      }

      // get function name
      esBuilder.setCommand(ctx.function().name().getText());

      // get input args
      esBuilder.setInputArgs(getInputArgs(ctx.function().inputList()));

      // get return arg (optional)
      List<String> returnArgs = new ArrayList<>();
      if (ctx.specReturn() != null && ctx.specReturn().outputList() != null && ctx.specReturn().outputList().output() != null) {
        returnArgs.add(ctx.specReturn().outputList().output().getText());
        ctx.specReturn().outputList().outputTail().forEach(tail -> returnArgs.add(tail.output().getText()));
      }
      esBuilder.setReturnArgs(returnArgs);

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterIfStatement(ASLParser.IfStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("if");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitIfStatement(ASLParser.IfStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endif");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterElifStatement(ASLParser.ElifStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("elseif");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterElseStatement(ASLParser.ElseStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("else");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterWhileStatement(ASLParser.WhileStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("while");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitWhileStatement(ASLParser.WhileStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endwhile");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterForStatement(ASLParser.ForStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("for");

      ASLParser.ForInitialContext initial = ctx.forInitial();
      String loopVariable = initial.name(0).getText();
      if (!loopVariable.equals(ctx.forCondition().name(0).getText())
              || !loopVariable.equals(ctx.forUpdate().name(0).getText())) {
        throw new IllegalArgumentException("For-loop initialization, condition, and update must use the same variable at "
                + ctx.getStart().getLine() + ":" + ctx.getStart().getCharPositionInLine());
      }
      esBuilder.addInputArg(loopVariable);
      if (initial.numberLiteral() != null) {
        esBuilder.addInputArg(initial.numberLiteral().getText());
      } else if (initial.name().size() > 1) {
        esBuilder.addInputArg(initial.name().get(1).getText());
      }

      ASLParser.ForConditionContext condition = ctx.forCondition();
      esBuilder.addInputArg(condition.CMPOPT().getText());
      if (condition.numberLiteral() != null) {
        esBuilder.addInputArg(condition.numberLiteral().getText());
      } else {
        esBuilder.addInputArg(condition.name().get(1).getText());
      }

      ASLParser.ForUpdateContext update = ctx.forUpdate();
      StringBuilder sb = new StringBuilder();
      if (update.INCOPT() != null) {
        sb.append(update.INCOPT().getText());
      } else {
        sb.append(update.GRWOPT().getText());
        if (update.numberLiteral() != null) {
          sb.append(update.numberLiteral().getText());
        } else {
          sb.append(update.name().get(1).getText());
        }
      }
      esBuilder.addInputArg(sb.toString());

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitForStatement(ASLParser.ForStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endfor");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterForEachStatement(ASLParser.ForEachStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL)
              .setCommand("foreach")
              .addInputArg(ctx.name(0).getText()) // element name
              .addInputArg(ctx.name(1).getText()); // list name
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitForEachStatement(ASLParser.ForEachStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endfor");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterTryStatement(ASLParser.TryStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("try");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitTryStatement(ASLParser.TryStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endtry");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterCatchStatement(ASLParser.CatchStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("catch");

      // collect optional input args
      ASLParser.CatchParameterContext cp = ctx.catchParameter();
      if (cp != null) {
        if (cp.NAME() != null) {
          esBuilder.addInputArg(cp.NAME().getText());
        }
        if (cp.output() != null) {
          esBuilder.addInputArg(cp.output().getText());
        }
      }

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterFinallyStatement(ASLParser.FinallyStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("finally");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterExitStatement(ASLParser.ExitStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("exit");
      esBuilder.setInputArgs(getInputArgs(ctx.inputList()));
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterReturnStatement(ASLParser.ReturnStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("return");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterAsyncStatement(ASLParser.AsyncStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("async");
      esBuilder.setInputArgs(getInputArgs(ctx.inputList()));
      if (ctx.singleArgReturn() != null) {
        esBuilder.addReturnArg(ctx.singleArgReturn().output().getText());
      }
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitAsyncStatement(ASLParser.AsyncStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endasync");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterJoinStatement(ASLParser.JoinStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("join");

      // input args
      esBuilder.setInputArgs(getInputArgs(ctx.inputList()));

      addReturns(esBuilder, ctx.specReturn());

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void enterInvokeStatement(ASLParser.InvokeStatementContext ctx) {
      EventSpec.Builder event = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("invoke")
          .setInputArgs(getInputArgs(ctx.inputList()));
      addReturns(event, ctx.specReturn());
      curr_dbe_builder.addEventSpec(event.build());
    }

    private void addReturns(EventSpec.Builder event, ASLParser.SpecReturnContext returns) {
      if (returns != null) {
        event.addReturnArg(returns.outputList().output().getText());
        returns.outputList().outputTail().forEach(tail -> event.addReturnArg(tail.output().getText()));
      }
    }

    @Override
    public void enterValueAssignment(ASLParser.ValueAssignmentContext ctx) {
      curr_dbe_builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
          .addInputArg(expressionText(ctx.valueExpression())).addReturnArg(ctx.output().getText()).build());
    }

    @Override
    public void enterOrExpression(ASLParser.OrExpressionContext ctx) {
      if (ctx.andExpression().size() > 1) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("or");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    @Override
    public void exitOrExpression(ASLParser.OrExpressionContext ctx) {
      if (ctx.andExpression().size() > 1) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endor");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    @Override
    public void enterAndExpression(ASLParser.AndExpressionContext ctx) {
      if (ctx.primaryExpression().size() > 1) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("and");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    @Override
    public void exitAndExpression(ASLParser.AndExpressionContext ctx) {
      if (ctx.primaryExpression().size() > 1) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endand");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    @Override
    public void enterPrimaryExpression(ASLParser.PrimaryExpressionContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL);
      if (ctx.valueExpression() != null) {
        esBuilder.setCommand("evaluate").addInputArg(expressionText(ctx.valueExpression()));
      } else if (ctx.TRUE() != null) {
        esBuilder.setCommand("true");
      } else if (ctx.FALSE() != null) {
        esBuilder.setCommand("false");
      } else if (ctx.NOT() != null) {
        esBuilder.setCommand("not");
      } else {
        return;
      }

      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void exitPrimaryExpression(ASLParser.PrimaryExpressionContext ctx) {
      if (ctx.NOT() != null) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endnot");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    @Override public void enterNoInterruptStatement(ASLParser.NoInterruptStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("nointerrupt");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override public void exitNoInterruptStatement(ASLParser.NoInterruptStatementContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endnointerrupt");
      curr_dbe_builder.addEventSpec(esBuilder.build());
    }

    @Override
    public void visitErrorNode(ErrorNode node) {
      log.warn("error parsing " + dbFile + " : " + node.getSymbol().getText() + " at line: " + node.getSymbol().getLine() + ":" + node.getSymbol().getCharPositionInLine());
    }

    private Predicate processPredicate(ASLParser.PredicateContext ctx) {

      List<String> args = new ArrayList<>();
      if (ctx.inputList().input() != null) {
        args.add(ctx.inputList().input().getText());
        ctx.inputList().inputTail().forEach(tail -> args.add((tail.input().getText())));
      }

      Predicate predicate = Factory.createPredicate(ctx.name().getText(), args.toArray(new String[0]));

      if (ctx.NOT() != null) {
        return predicate.toNegatedForm();
      } else {
        return predicate;
      }
    }

    /**
     * Get the semantic type for the specified role.
     *
     * @param role_name
     * @return
     */
    private String getSemanticType(String role_name) {
      ActionBinding adbe_role = curr_dbe_builder.getRoles().stream().filter(role -> role.getName().equals(role_name)).findFirst().orElse(null);
      if (adbe_role != null && adbe_role.hasSemanticType()) {
        return adbe_role.getSemanticType();
      } else if (role_name.equals("?actor")) {
        // ?actor role is not usually explicitly defined and isn't added to the ActionDBEntry until it's built
        return curr_dbe_builder.getDefaultActorType();
      } else {
        return null;
      }
    }

    private void addSignatureRole(ASLParser.ParameterContext ctx, boolean roleReturn) {
      addRole(ctx.GLOBALVAR().getText(), ctx.javaType().getText(), ctx.assignment(), false, roleReturn);
    }

    private void emitBlockLocalDeclaration(String roleName, String semanticType, String javaType, ASLParser.AssignmentContext assignmentContext) {
      Class<?> varType = resolveJavaTypeOrThrow(javaType);
      curr_dbe_builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("declare")
          .addInputArg(roleName).addInputArg(varType.getCanonicalName()).addInputArg(semanticType == null ? "" : semanticType).build());

      if (assignmentContext == null) {
        return;
      }
      if (assignmentContext.valueExpression() != null) {
        curr_dbe_builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
            .addInputArg(expressionText(assignmentContext.valueExpression())).addReturnArg(roleName).build());
      } else if (assignmentContext.input() != null) {
        curr_dbe_builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
            .addInputArg(assignmentContext.input().getText()).addReturnArg(roleName).build());
      } else if (assignmentContext.specType() != null) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(getEventType(assignmentContext.specType()));
        if (assignmentContext.specType().name() != null) {
          esBuilder.setActor(assignmentContext.specType().name().getText());
        }
        esBuilder.setCommand(assignmentContext.function().name().getText());
        esBuilder.setInputArgs(getInputArgs(assignmentContext.function().inputList()));
        esBuilder.addReturnArg(roleName);
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
    }

    private Class<?> resolveJavaTypeOrThrow(String javaType) {
      String typeName = resolveTypeNameForLookup(javaType);
      Class<?> varType = Utilities.getClass(typeName);
      if (varType == null) {
        throw new IllegalArgumentException("Class not found for type: " + typeName);
      }
      return varType;
    }

    private String resolveTypeNameForLookup(String javaType) {
      String typeName = aliasingMap.getOrDefault(javaType, javaType);
      int genericStart = typeName.indexOf('<');
      if (genericStart >= 0) {
        typeName = typeName.substring(0, genericStart).trim();
      }
      return aliasingMap.getOrDefault(typeName, typeName);
    }

    private ParsedVar parseVar(String token) {
      int idx = token.indexOf(':');
      if (idx < 0) {
        return new ParsedVar(token, "");
      }
      return new ParsedVar(token.substring(0, idx), token.substring(idx + 1));
    }

    /**
     * Helper method to add input/output roles and local roles.
     *
     * @param roleName
     * @param javaType
     * @param assignmentContext
     * @param isLocal
     * @param isReturn
     */
    private void addRole(String roleName, String javaType, ASLParser.AssignmentContext assignmentContext, boolean isLocal, boolean isReturn) {
      String typeName = resolveTypeNameForLookup(javaType);
      Class<?> varType = Utilities.getClass(typeName);
      if (varType == null) {
        throw new IllegalArgumentException("Class not found for type: " + typeName + " (role " + roleName + ")");
      }

      ActionBinding.Builder newRole = new ActionBinding.Builder(roleName, varType).setIsLocal(isLocal).setIsReturn(isReturn);

      if (assignmentContext != null) {
        if (assignmentContext.valueExpression() != null) {
          if (!isLocal) throw new IllegalArgumentException("Signature defaults must be literals");
          curr_dbe_builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("assign")
              .addInputArg(expressionText(assignmentContext.valueExpression())).addReturnArg(roleName).build());
        } else if (assignmentContext.input() != null) {
          // add default value
          newRole.setDefaultValue(stripQuotes(assignmentContext.input().getText()));
        } else if (assignmentContext.specType() != null) {
          // add event spec to assign value during execution
          EventSpec.Builder esBuilder = new EventSpec.Builder(getEventType(assignmentContext.specType()));

          // get actor (optional)
          if (assignmentContext.specType().name() != null) {
            esBuilder.setActor(assignmentContext.specType().name().getText());
          }

          // get command
          esBuilder.setCommand(assignmentContext.function().name().getText());

          // input args
          esBuilder.setInputArgs(getInputArgs(assignmentContext.function().inputList()));

          // return arg
          esBuilder.addReturnArg(roleName);

          curr_dbe_builder.addEventSpec(esBuilder.build());
        }
      }

      curr_dbe_builder.addRole(newRole.build());
    }

    /**
     * Helper method to extract input arguments from ASL into a List.
     *
     * @param input
     * @return
     */
    private List<String> getInputArgs(ASLParser.InputListContext input) {
      List<String> inputArgs = new ArrayList<>();
      if (input != null && input.input() != null) {
        inputArgs.add(input.input().getText());
        input.inputTail().forEach(tail -> inputArgs.add(tail.input().getText()));
      }
      return inputArgs;
    }

    /**
     * Convert ANTLR SpecType to EventType.
     *
     * @param specType
     * @return
     */
    private EventSpec.EventType getEventType(ASLParser.SpecTypeContext specType) {
      if (specType.ACTION() != null) {
        return EventSpec.EventType.ACTION;
      } else if (specType.OPERATOR() != null) {
        return EventSpec.EventType.OPERATOR;
      } else if (specType.OBSERVATION() != null) {
        return EventSpec.EventType.OBSERVATION;
      } else if (specType.GOAL() != null) {
        return EventSpec.EventType.GOAL;
      } else if (specType.TSC() != null) {
        return EventSpec.EventType.TSC;
      } else if (specType.THIS() != null) {
        return EventSpec.EventType.THIS;
      } else {
        log.error("Unexpected EventType in event spec: " + specType.getText());
        return null;
      }
    }

    /**
     * Convert ANTLR InterruptSpecType to EventType.
     *
     * @param specType
     * @return
     */
    private EventSpec.EventType getEventType(ASLParser.InterruptSpecTypeContext specType) {
      if (specType.ACTION() != null) {
        return EventSpec.EventType.ACTION;
      } else if (specType.GOAL() != null) {
        return EventSpec.EventType.GOAL;
      } else if (specType.TSC() != null) {
        return EventSpec.EventType.TSC;
      } else {
        log.error("Unexpected EventType in event spec: " + specType.getText());
        return null;
      }
    }

    private String stripQuotes(String s) {
      if (s == null || s.charAt(0) != '"' || s.charAt(s.length() - 1) != '"') {
        return s;
      }
      return s.substring(1, s.length() - 1);
    }
  }
}
