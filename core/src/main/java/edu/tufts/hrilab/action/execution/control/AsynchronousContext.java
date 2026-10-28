/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.execution.AslTask;
import edu.tufts.hrilab.action.asl.AslValues;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;
import java.util.List;
import java.util.LinkedHashMap;
import java.util.Map;

public class AsynchronousContext extends ArgumentBasedContext {
  private ScriptParser parser;
  private ScriptParser currParserStep;
  private AslTask task;
  private final Map<String, Class<?>> exports = new LinkedHashMap<>();
  private boolean taskHandle;

  public AslTask getTask() { return task; }
  public boolean hasTaskHandle() { return taskHandle; }

  public boolean exports(String name) { return exports.containsKey(name); }

  public AsynchronousContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "ASYNC");
    setupArguments(parser.getEventSpec().getInputArgs(), parser.getEventSpec().getReturnArgs());
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  public boolean isAsynchronous() {
    return true;
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    for (Object input : inputArgs) {
      String name = input.toString();
      Class<?> type = getArgumentType(name);
      if (!Utilities.isScriptVariable(name) || type == null || exports.putIfAbsent(name, type) != null) {
        throw new IllegalArgumentException("Async exports must be distinct declared variables: " + name);
      }
      ActionBinding binding = new ActionBinding.Builder(name, type)
          .setValue(getArgumentSilent(name).getBindingDeep()).setIsLocal(true).build();
      arguments.put(name, binding);
    }
    task = new AslTask(this, exports);

    if (returnArgs.size() == 1) {
      if (!Utilities.isScriptVariable(returnArgs.get(0).toString())) {
        log.error("[setupArguments] invalid arg. Must be a variable. arg: " + returnArgs.get(0));
        return;
      }
      Class<?> handleType = AslValues.boxed(getArgumentType(returnArgs.get(0).toString()));
      if (handleType != Long.class && handleType != AslTask.class) {
        throw new IllegalArgumentException("Async return must be Long or AslTask");
      }
      if (!exports.isEmpty() && handleType != AslTask.class) {
        throw new IllegalArgumentException("Async exports require an AslTask handle");
      }
      addArgument("asyncId", handleType, returnArgs.get(0), false, true);
      setArgument("asyncId", handleType == Long.class ? getId() : task);
      taskHandle = handleType == AslTask.class;
    } else if (returnArgs.size() > 1) {
      throw new IllegalArgumentException("Async accepts at most one handle");
    } else if (!exports.isEmpty()) {
      throw new IllegalArgumentException("Async exports require an AslTask handle");
    }

    if (returnArgs.size() == 1 && AslTask.class.equals(getArgumentType(returnArgs.get(0).toString()))) {
      for (Context scope = caller; scope != null; scope = scope.getParentContext()) {
        if (scope instanceof ArgumentBasedContext) {
          for (ActionBinding source : ((ArgumentBasedContext) scope).getLocalArgumentBindings()) {
            if (Utilities.isScriptVariable(source.getName())) {
              arguments.putIfAbsent(source.getName(), new ActionBinding.Builder(source.getName(), source.getJavaType())
                  .setValue(source.getBindingDeep()).setIsLocal(true).build());
            }
          }
        }
        if (scope.isAction()) break;
      }
    }

    // zero return args is acceptable, no additional work needed
  }

  @Override
  protected void setupNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  @Override
  public AsynchronousContext copy(Context newParent) {
    AsynchronousContext copy = new AsynchronousContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(copy);
    return copy;
  }
}
