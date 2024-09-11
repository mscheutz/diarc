/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.db.OperatorDBEntry;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.*;
import java.util.stream.Collectors;

public class TradeServiceContext extends ArgumentBasedContext {

  /**
   * Cache of input arg java type for TRADE Service lookup. Populated during setupArgs.
   */
  private List<Class<?>> inputArgTypes = new ArrayList<>();

  protected TradeServiceContext(Context caller, StateMachine sm, String cmd, List<String> inputArgs, List<String> returnArgs, Symbol actor) {
    super(caller, sm, cmd);
    addArgument("?actor", Symbol.class, actor, false, false); // add ?actor as first arg in context
    setupArguments(inputArgs, returnArgs);
  }

  @Override
  protected void setupArguments(List<?> inputArgs, List<?> returnArgs) {
    // setup input args
    int index = 0;
    for (Object arg : inputArgs) {
      Class cls = getArgumentType((String) arg);
      if (cls == null) {
        log.warn("[setupArguments] null role type returned for argument: " + arg + " Defaulting to String type.");
        cls = String.class;
      }
      inputArgTypes.add(cls);
      addArgument("var" + index++, cls, arg, false, false);
    }

    // setup return arg
    if (!returnArgs.isEmpty()) {
      Class cls = getArgumentType((String) returnArgs.get(0));
      addArgument("ret", cls, returnArgs.get(0), false, true);
    } else {
      //
      addArgument("ret", Object.class, null, false, true);
    }

  }

  @Override
  public void doStep() {
    TRADEServiceInfo tsi = getTSI();
    if (tsi == null) {
      Symbol actor = (Symbol) getArgument("?actor").getBinding();
      Justification just = new ConditionJustification(false, Factory.createPredicate("found", actor, getSignatureInPredicateForm()));
      setStatus(ActionStatus.FAIL_NOTFOUND, just);
    } else {
      callTSI(tsi, getArguments());
      redistributeArguments();
    }

  }

  private TRADEServiceInfo getTSI() {
    TRADEServiceConstraints constraints = new TRADEServiceConstraints().name(cmd).argTypes(inputArgTypes.toArray(new Class<?>[0]));
    Collection<TRADEServiceInfo> options = TRADE.getAvailableServices(constraints);

    // filter options based on agent group
    String agent = "agent:" + getArgument("?actor").getBinding().toString();
    List<TRADEServiceInfo> filteredOptions = options.stream().filter(tsi -> tsi.getGroups().isEmpty() || containsAgent(tsi.getGroups(),agent)).toList();
    if (filteredOptions.size() == 1) {
      return filteredOptions.get(0);
    } else {
      log.error("[doStep] cannot find unique TSI for: " + constraints+" found: "+filteredOptions);
      return null;
    }
  }

  /**
   * Helper method to check if TRADE groups contains the diarc agent (e.g., agent:dempster or agent:dempster:nao).
   *
   * @param groups TRADE groups
   * @param agent DIARC agent --> agent:name(:semanticType)
   * @return true if groups contains agent
   */
  private boolean containsAgent(Collection<String> groups, String agent) {
    for (String group : groups) {
      if (group.startsWith(agent)) {
        return true;
      }
    }
    return false;
  }

  private Justification callTSI(TRADEServiceInfo tsi, Collection<ActionBinding> args) {
    log.debug("callTSI with " + tsi);

    // collect non-return args and find return arg to be filled
    List<Object> argBindings = new ArrayList();
    ActionBinding returnArg = null;
    boolean firstArg = true;
    if (args != null) {
      for (ActionBinding arg : args) {
        if (firstArg) {
          // first arg is ?actor -- leave out of TSI call
          firstArg = false;
          continue;
        } else if (arg.isReturn) {
          if (returnArg != null) {
            log.error("More than one return value in executeAction for primitive action: " + tsi);
          }
          returnArg = arg;
        } else {
          if (!arg.isBound()) {
            log.error("Trying to call executeAction with unbound arguments for primitive action: " + tsi);
          }
          Object value = arg.getBindingDeep();
          argBindings.add(value);
        }
      }
    }

    Justification executionJustification;
    try {
      // make call to primitive action
      Object returnValue = tsi.call(Object.class, argBindings.toArray());
      // bind return value
      if (returnArg == null) {
        log.debug("No designated return argument in executeAction for primitive action: " + tsi);
      } else {
        returnArg.bindDeep(returnValue);
      }
      executionJustification = new ConditionJustification(true);
    } catch (TRADEException e) {
      log.error("Exception while calling TSI " + tsi + " for : " + cmd + " with args : " + args, e);
      executionJustification = new ConditionJustification(false, Factory.createPredicate("succeed", tsi.serviceName));
    }
    return executionJustification;
  }


  /**
   * Private constructor for copy method.
   *
   * @param caller
   * @param sm
   * @param cmd
   * @param arguments
   */
  private TradeServiceContext(Context caller, StateMachine sm, String cmd, LinkedHashMap<String, ActionBinding> arguments) {
    super(caller, sm, cmd);
    copyArguments(arguments);
  }

  @Override
  public Context copy(Context newParent) {
    TradeServiceContext tsContext = new TradeServiceContext(newParent, newParent.stateMachine, cmd, this.arguments);
    tsContext.inputArgTypes = new ArrayList<>(this.inputArgTypes);
    copyInternal(tsContext);
    return tsContext;
  }
}
