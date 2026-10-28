/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.asl.AslExpression;
import edu.tufts.hrilab.action.asl.AslValues;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.state.StateMachine;

public final class ExpressionContext extends Context {
  private final EventSpec event;
  private final AslExpression expression;
  private final java.util.Map<String, ActionBinding> snapshots = new java.util.HashMap<>();

  public ExpressionContext(Context parent, StateMachine state, EventSpec event) {
    super(parent, state, event.getCommand());
    this.event = event;
    expression = new AslExpression(event.getInputArgs().get(0));
    if (parent instanceof AsynchronousContext && !((AsynchronousContext) parent).hasTaskHandle()) {
      java.util.Set<String> variables = new java.util.HashSet<>(expression.variables());
      variables.addAll(event.getReturnArgs());
      for (String name : variables) {
        ActionBinding binding = super.getArgumentSilent(name);
        if (binding != null) {
          snapshots.put(name, new ActionBinding.Builder(name, binding.getJavaType())
              .setValue(binding.getBindingDeep()).build());
        }
      }
    }
  }

  @Override
  protected ActionBinding getArgumentSilent(String name) {
    ActionBinding binding = snapshots.get(name);
    return binding == null ? super.getArgumentSilent(name) : binding;
  }

  @Override
  public void doStep() {
    try {
      Object value = expression.evaluate(this);
      if (event.getReturnArgs().isEmpty()) {
        if (!(value instanceof Boolean)) throw new IllegalArgumentException("ASL condition must be Boolean");
        setLogicalValue((Boolean) value);
      } else {
        ActionBinding target = getArgumentSilent(event.getReturnArgs().get(0));
        if (target == null) throw new IllegalArgumentException("Assignment target is undeclared");
        target.bindDeep(AslValues.convert(value, target.getJavaType()));
      }
    } catch (IllegalArgumentException | ArithmeticException exception) {
      failExecution(ActionStatus.FAIL_ARGUMENTS, exception.getMessage());
    }
  }

  @Override
  public Context copy(Context parent) {
    ExpressionContext copy = new ExpressionContext(parent, parent.getStateMachine(), event);
    copy.snapshots.clear();
    snapshots.forEach((name, binding) -> copy.snapshots.put(name, binding.clone()));
    copyInternal(copy);
    return copy;
  }
}
