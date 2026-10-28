/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.asl.CompiledAction;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import java.util.List;

public final class InvokeContext extends Context {
  private final EventSpec event;
  private boolean started;

  public InvokeContext(Context parent, StateMachine state, EventSpec event) {
    super(parent, state, "invoke");
    this.event = event;
  }

  @Override
  protected void setupNextStep() {
    if (started) return;
    started = true;
    List<String> inputs = event.getInputArgs();
    try {
      if (inputs.size() < 2) throw new IllegalArgumentException("Invoke requires a compiled module and actor");
      Object value = getArgumentValue(inputs.get(0));
      if (!(value instanceof CompiledAction)) throw new IllegalArgumentException("Invoke requires a CompiledAction");
      CompiledAction compiled = (CompiledAction) value;
      String actorInput = inputs.get(1);
      Object actor = actorInput.startsWith("?") || actorInput.startsWith("!")
          ? getArgumentValue(actorInput) : Factory.createSymbol(actorInput);
      if (!(actor instanceof Symbol)) throw new IllegalArgumentException("Invoke actor must be a Symbol");
      ActionDBEntry action = compiled.getEntry();
      int count = inputs.size() - 2;
      var roles = action.getInputRoles();
      if (count > roles.size() - 1) throw new IllegalArgumentException("Too many compiled action inputs");
      for (int i = count + 1; i < roles.size(); i++) {
        if (roles.get(i).getDefaultValue() == null) throw new IllegalArgumentException("Missing compiled action input");
      }
      if (event.getReturnArgs().size() > action.getReturnRoles().size()) {
        throw new IllegalArgumentException("Too many compiled action outputs");
      }
      addCompiledAction(compiled, inputs.subList(2, inputs.size()), event.getReturnArgs(), (Symbol) actor);
    } catch (IllegalArgumentException exception) {
      failExecution(ActionStatus.FAIL_ARGUMENTS, exception.getMessage());
    }
  }

  @Override
  public Context copy(Context parent) {
    InvokeContext copy = new InvokeContext(parent, parent.getStateMachine(), event);
    copy.started = started;
    copyInternal(copy);
    return copy;
  }
}
