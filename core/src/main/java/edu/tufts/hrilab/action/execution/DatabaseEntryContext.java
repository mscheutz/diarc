/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.db.DBEntry;

import java.util.*;

public abstract class DatabaseEntryContext<T extends DBEntry> extends ArgumentBasedContext {

  /**
   * <code>dbe</code> is the definition for the DBEntry that is being tracked by
   * this context. All of the context information pertaining to the invocation
   * of that DBentry are held in this class.
   */
  private T dbe;

  protected DatabaseEntryContext(Context caller, StateMachine sm, T dbentry) {
    super(caller, sm, dbentry.getType());
    dbe = dbentry;
  }

  protected DatabaseEntryContext(Context caller, StateMachine sm, T dbentry, ExecutionType executionType) {
    super(caller, sm, dbentry.getType(), executionType);
    dbe = dbentry;
  }

  /**
   * Create predicate from the context signature.
   * @return
   */
  public Predicate getSignatureInPredicateForm() {
    List<Symbol> args = new ArrayList<>();
    for (ActionBinding arg : this.collectArguments()) {
      if (arg.isReturn() || arg.isLocal) {
        // return args are at the end and we don't want them here, so we're done
        break;
      } else if (arg.isBound()) {
        Object val = arg.getBindingDeep();
        if (val == null) {
          args.add(Factory.createFOL(arg.name));
        } else if (val instanceof Symbol) {
          args.add((Symbol) val);
        } else if (val != null) {
          //TODO:brad: see other comment about getting a single predicate directly from a justification
          args.add(Factory.createFOL("\"" + val + "\""));
        } else {
          log.error("null value in signature? for context: " + this);
        }
      } else {
        args.add(Factory.createFOL(arg.name));
      }
    }
    return new Predicate(this.cmd, args);
  }

  /**
   * @return the DB Entry used in this context.
   */
  public T getDBE() {
    return dbe;
  }

  /**
   * @param dbe DB Entry to be used in this context.
   * @return true if successfully set dbe otherwise false
   */
  public boolean setDBE(T dbe) {
    if (this.dbe == null) {
      this.dbe = dbe;
      return true;
    }
    log.warn("[setDBE] cannot set the dbe to " + dbe + " because it is already bound");
    return false;
  }

  /**
   * Add an argument to the list of arguments that were passed when the script
   * was invoked.
   *
   * @param roleIndex the position at which to add the argument
   * @param value the value to be assigned to the argument
   * @return true if successful, false otherwise
   */
  protected boolean addArgument(int roleIndex, Object value) {
    String name = dbe.getRoleName(roleIndex);
    Class<?> javaType = dbe.getRoleJavaType(roleIndex);
    String semanticType = dbe.getRoleSemanticType(roleIndex);
    String defaultValue = dbe.getRoleDefault(roleIndex);
    boolean isLocal = dbe.getRoleLocal(roleIndex);
    boolean isReturn = dbe.getRoleReturn(roleIndex);
    boolean isVarArg = dbe.getRoleIsVarArg(roleIndex);

    return addArgument(name, javaType, semanticType, value, defaultValue, isLocal, isReturn, isVarArg);
  }

  /**
   * Add a varArg argument to the Context, using the DBEntry role as a guide to
   * populate the argument details.
   * @param roleIndex
   * @param value
   * @param varArgCount
   * @return
   */
  protected boolean addVarArgArgument(int roleIndex, Object value, int varArgCount) {
    String name = dbe.getRoleName(roleIndex) + varArgCount;
    Class<?> javaType = dbe.getRoleJavaType(roleIndex).getComponentType();
    String semanticType = dbe.getRoleSemanticType(roleIndex);
    String defaultValue = dbe.getRoleDefault(roleIndex);
    boolean isLocal = dbe.getRoleLocal(roleIndex);
    boolean isReturn = dbe.getRoleReturn(roleIndex);
    boolean isVarArg = dbe.getRoleIsVarArg(roleIndex);

    return addArgument(name, javaType, semanticType, value, defaultValue, isLocal, isReturn, isVarArg);
  }

  /**
   * Specify the (deep) value of an argument
   *
   * @param pos the position at which to add the argument
   * @param val the value to which the argument should be bound
   */
  protected void setArgument(int pos, Object val) {
    String bName = dbe.getRoleName(pos);
    setArgument(bName, val);
  }

  /**
   * Goes through and binds the values in an event spec to arguments.
   */
  protected void setupArguments(Map<String, Object> bindings) {
    // Find bindings for each role
    for (int i = 0; i < dbe.getNumRoles(); i++) {
      String name = dbe.getRoleName(i);
      if (bindings.containsKey(name)) {
        Object val = bindings.get(name);
        if (val != null) {
          log.debug(getCommand() + ": Adding argument " + name + " = " + bindings.get(name).toString());
        } else {
          log.debug(getCommand() + ": Adding argument " + name + " = " + null);
        }
        this.addArgument(i, bindings.get(name));
      } else {
        // no binding passed in for role, use null for its current value
        log.debug(getCommand() + ": Adding argument by role " + name);
        this.addArgument(i, null);
      }
    }

    // Add other bindings as context variables
    for (String name : bindings.keySet()) {
      if (!this.hasLocalArgument(name)) {
        Class<?> clazz = getArgumentType(name);
        Object value = bindings.get(name);
        this.addArgument(name, clazz, value, true, false);
      }
    }
  }

  /**
   * Goes through and binds the values in an event spec to arguments. Interprets
   * each element according to its role.
   */
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    int roleIndex = 0;

    // handle input roles
    int numInputRoles = dbe.getInputRoles().size();
    for (int i = 0; i < numInputRoles; i++) {

      if (dbe.getRoleIsVarArg(roleIndex)) {
        // the rest of the input args are for the var args role
        int varIdx = 0;
        for (int j = i; j < inputArgs.size(); j++) {
          log.debug(getCommand() + ": Adding argument " + inputArgs.get(j));
          this.addVarArgArgument(roleIndex, inputArgs.get(j), varIdx++);
        }
      } else {
        // handle non-varArg roles
        if (i < inputArgs.size()) {
          log.debug(getCommand() + ": Adding argument " + inputArgs.get(i).toString());
          this.addArgument(roleIndex, inputArgs.get(i));
        } else {
          // no arg passed in for role, use null for its current value
          String name = dbe.getRoleName(roleIndex);
          log.debug(getCommand() + ": Adding argument by role " + name);
          this.addArgument(roleIndex, null);
        }
      }
      ++roleIndex;
    }

    // handle return args
    int numReturnRoles = dbe.getReturnRoles().size();
    for (int i = 0; i < numReturnRoles; i++) {
      if (i < returnArgs.size()) {
        log.debug(getCommand() + ": Adding argument " + returnArgs.get(i).toString());
        this.addArgument(roleIndex, returnArgs.get(i));
      } else {
        // no arg passed in for role, use null for its current value
        String name = dbe.getRoleName(roleIndex);
        log.debug(getCommand() + ": Adding argument by role " + name);
        this.addArgument(roleIndex, null);
      }
      ++roleIndex;
    }

    // add local args to context (can't be set by input or return args)
    int numLocalRoles = dbe.getLocalRoles().size();
    for (int i = 0; i < numLocalRoles; i++) {
        // use null for all local vars
        String name = dbe.getRoleName(roleIndex);
        log.debug(getCommand() + ": Adding argument by role " + name);
        this.addArgument(roleIndex, null);
        ++roleIndex;
    }


    // sanity check
    if (dbe.isVarArgs() && arguments.size() < dbe.getNumRoles()-1
            || !dbe.isVarArgs() && arguments.size() != dbe.getNumRoles()) {
      log.error("ERROR: event signature mismatch:" + dbe.getName());
      if (log.isDebugEnabled()) {
        log.debug("    " + dbe.getName() + " expects: ");
        for (int i = 0; i < dbe.getNumRoles(); i++) {
          log.debug(" " + dbe.getRoleName(i));
        }
        log.debug(",\n    but got: ");
        for (Object i : inputArgs) {
          log.debug(" " + i.toString());
        }
        log.debug(" instead");
      }
    }
  }

  /**
   * Collect arguments (not local vars) in the correct order to be passed into the primitive
   * action method. This should only return the required args of primitive and return arg
   *
   * @return ordered primitive action arguments
   */
  public Collection<ActionBinding> collectArguments() {
    Collection<ActionBinding> args = new ArrayList<>();
    if (dbe == null) {
      return args;
    }

    arguments.forEach((name, binding) -> {
      if (!binding.isLocal()) {
        args.add(getEvaluatedArgument(name));
      }
    });
    
    return args;
  }

}
