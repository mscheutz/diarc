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
   * @return
   */
  public List<ActionDBEntry> parseFromFile(String dbFile, List<Symbol> agents) {
    log.debug("Parsing action db file: " + dbFile + " for agents: " + agents);
    this.dbFile = dbFile;
    this.agents = agents;

    try {
      // LEX THE INPUT
      CharStream input = CharStreams.fromStream(getClass().getResourceAsStream(dbFile));
      ASLLexer lexer = new ASLLexer(input);
      CommonTokenStream tokens = new CommonTokenStream(lexer);

      // PARSE THE TOKENS
      ASLParser parser = new ASLParser(tokens);
      parser.setBuildParseTree(true);
      ParseTree tree = parser.actionScript();

      // WALK THE PARSE TREE
      ParseTreeWalker walker = new ParseTreeWalker();
      AsEmitter converter = new AsEmitter();
      walker.walk(converter, tree);
      log.debug("Successfully parsed " + dbFile + " !");
      return converter.getParsedAction();
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
   * @return
   */
  public List<ActionDBEntry> parseFromString(String dbFile, List<Symbol> agents, String asl) {
    log.debug("[parseFromString] Attempting to create ActionDBEntrys from asl: " + asl);
    this.dbFile = dbFile;
    this.agents = agents;

    try {
      // LEX THE INPUT
      CharStream input = CharStreams.fromString(asl);
      ASLLexer lexer = new ASLLexer(input);
      CommonTokenStream tokens = new CommonTokenStream(lexer);

      // PARSE THE TOKENS
      ASLParser parser = new ASLParser(tokens);
      parser.setBuildParseTree(true);
      ParseTree tree = parser.actionScript();

      // WALK THE PARSE TREE
      ParseTreeWalker walker = new ParseTreeWalker();
      AsEmitter converter = new AsEmitter();
      walker.walk(converter, tree);
      log.debug("Successfully parsed " + dbFile + " !");
      return converter.getParsedAction();
    } catch (Exception e) {
      log.error("Error parsing file: " + dbFile, e);
      return null;
    }
  }

  private class AsEmitter extends ASLBaseListener {

    private Builder curr_dbe_builder;
    private List<ActionDBEntry> parsed_actions;

    private Map<String, String> aliasingMap;
    private Condition.Disjunction orCondition = null;

    List<ActionDBEntry> getParsedAction() {
      return parsed_actions;
    }

    @Override
    public void enterActionScript(ASLParser.ActionScriptContext ctx) {
      aliasingMap = new HashMap<>();
      parsed_actions = new ArrayList<>();
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
      String methodName = ctx.NAME().getText();
      curr_dbe_builder = new Builder(methodName);
      curr_dbe_builder.setDBFile(dbFile);
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
      parsed_actions.add(curr_dbe_builder.build(false));
    }

    @Override
    public void enterDescription(ASLParser.DescriptionContext ctx) {
      String desc = stripQuotes(ctx.SENTENCE().getText());
      curr_dbe_builder.setDescription(desc);
    }

    @Override
    public void enterLocalvar(ASLParser.LocalvarContext ctx) {
      if (ctx.LOCALVAR() != null) {
        addRole(ctx.LOCALVAR().getText(), ctx.javaType().getText(), ctx.assignment(), true, false);
      } else {
        addRole(ctx.GLOBALVAR().getText(), ctx.javaType().getText(), ctx.assignment(), true, false);
      }
    }

    @Override
    public void enterSetting(ASLParser.SettingContext ctx) {
      String attribute = ctx.NAME().getText();
      String value = ctx.NUMBER().getText();
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
      if (ctx.recoveryGoals() != null) {
        ctx.recoveryGoals().predicateList().predicate().forEach(pred -> policyConstraints.goals.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryExcludedGoals() != null) {
        ctx.recoveryExcludedGoals().predicateList().predicate().forEach(pred -> policyConstraints.excludedGoals.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryFailedActions() != null) {
        ctx.recoveryFailedActions().predicateList().predicate().forEach(pred -> policyConstraints.failedActions.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryExcludedFailedActions() != null) {
        ctx.recoveryExcludedFailedActions().predicateList().predicate().forEach(pred -> policyConstraints.excludedFailedActions.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryFailureReasons() != null) {
        ctx.recoveryFailureReasons().predicateList().predicate().forEach(pred -> policyConstraints.failureReasons.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryExcludedFailureReasons() != null) {
        ctx.recoveryExcludedFailureReasons().predicateList().predicate().forEach(pred -> policyConstraints.excludedFailureReasons.add(Factory.createPredicate(pred.getText())));
      }
      if (ctx.recoveryActionStatuses() != null) {
        ctx.recoveryActionStatuses().NAME().forEach(status -> policyConstraints.actionStatuses.add(ActionStatus.fromString(status.getText())));
      }

      curr_dbe_builder.addRecoveryPolicyConstraint(policyConstraints);
    }

    @Override
    public void enterLocks(ASLParser.LocksContext ctx) {
      curr_dbe_builder.addResourceLock(ctx.name().getText());
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
      esBuilder.addInputArg(initial.name().get(0).getText());
      if (initial.NUMBER() != null) {
        esBuilder.addInputArg(initial.NUMBER().getText());
      } else if (initial.name().size() > 1) {
        esBuilder.addInputArg(initial.name().get(1).getText());
      }

      ASLParser.ForConditionContext condition = ctx.forCondition();
      esBuilder.addInputArg(condition.CMPOPT().getText());
      if (condition.NUMBER() != null) {
        esBuilder.addInputArg(condition.NUMBER().getText());
      } else {
        esBuilder.addInputArg(condition.name().get(1).getText());
      }

      ASLParser.ForUpdateContext update = ctx.forUpdate();
      StringBuilder sb = new StringBuilder();
      if (update.INCOPT() != null) {
        sb.append(update.INCOPT().getText());
      } else {
        sb.append(update.GRWOPT().getText());
        if (update.NUMBER() != null) {
          sb.append(update.NUMBER().getText());
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

      if (ctx.singleArgReturn() != null) {
        esBuilder.addReturnArg(ctx.singleArgReturn().output().getText());
      }

      curr_dbe_builder.addEventSpec(esBuilder.build());
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
      if (ctx.TRUE() != null) {
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

    @Override
    public void enterControlContent(ASLParser.ControlContentContext ctx) {
      EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL);
      ParserRuleContext r = ctx.getParent();
      if (r instanceof ASLParser.IfStatementContext || r instanceof ASLParser.ElifStatementContext) {
        esBuilder.setCommand("then");
      } else if (r instanceof ASLParser.WhileStatementContext) {
        esBuilder.setCommand("do");
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
      if (r instanceof ASLParser.ForStatementContext || r instanceof ASLParser.ForEachStatementContext) {
        EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("endblock");
        curr_dbe_builder.addEventSpec(esBuilder.build());
      }
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
      String typeName = aliasingMap.containsKey(javaType) ? aliasingMap.get(javaType) : javaType;
      Class<?> varType = Utilities.getClass(typeName);
      if (varType == null) {
        log.error("Class not found found for type: " + typeName + ". Role (" + roleName + ") will use default to String representation.");
        varType = String.class;
      }

      ActionBinding.Builder newRole = new ActionBinding.Builder(roleName, varType).setIsLocal(isLocal).setIsReturn(isReturn);

      if (assignmentContext != null) {
        if (assignmentContext.input() != null) {
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
