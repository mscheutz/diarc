/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.util.Utilities;
import java.util.List;

/** Declares a block-local variable at runtime. */
public final class DeclarationContext extends Context {
  public static final String COMMAND = "declare";
  private final EventSpec event;

  public DeclarationContext(Context parent, StateMachine state, EventSpec event) {
    super(parent, state, COMMAND);
    this.event = event;
  }

  @Override
  public void doStep() {
    try {
      List<String> args = event.getInputArgs();
      if (args.size() < 2) {
        throw new IllegalArgumentException("Declaration requires at least name and type");
      }
      String name = args.get(0);
      String typeName = args.get(1);
      String semanticType = args.size() > 2 ? args.get(2) : null;
      if (!Utilities.isScriptVariable(name)) {
        throw new IllegalArgumentException("Invalid declaration name: " + name);
      }
      Class<?> javaType = Utilities.getClass(typeName);
      if (javaType == null) {
        throw new IllegalArgumentException("Unsupported declared type: " + typeName);
      }
      ArgumentBasedContext scope = findScope();
      if (!scope.declareLocalArgument(name, javaType, semanticType)) {
        throw new IllegalArgumentException("Duplicate declaration in same scope: " + name);
      }
    } catch (IllegalArgumentException exception) {
      failExecution(ActionStatus.FAIL_ARGUMENTS, exception.getMessage());
    }
  }

  private ArgumentBasedContext findScope() {
    if (caller instanceof ArgumentBasedContext) {
      return (ArgumentBasedContext) caller;
    }
    throw new IllegalArgumentException("No lexical scope available for declaration");
  }

  @Override
  public Context copy(Context parent) {
    DeclarationContext copy = new DeclarationContext(parent, parent.getStateMachine(), event);
    copyInternal(copy);
    return copy;
  }
}
