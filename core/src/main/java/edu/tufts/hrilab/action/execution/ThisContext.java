package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.operators.StandardJavaMethods;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Symbol;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class ThisContext extends ArgumentBasedContext {
  public ThisContext(Context caller, StateMachine sm, String cmd, List<? extends Object> inputArgs, List<? extends Object> returnArgs, Symbol actor) {
    super(caller, sm, cmd);
    addArgument("?actor", Symbol.class, actor, false, false); // add ?actor as first arg in context
    setupArguments(inputArgs, returnArgs);
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    inputArgs.forEach(arg -> {
      ActionBinding actionBinding = new ActionBinding.Builder(arg.toString(), arg instanceof String ? getArgumentType((String)arg) : arg.getClass()).setIsLocal(true).build();
      this.arguments.put(actionBinding.getName(), actionBinding);
    });
    returnArgs.forEach(arg -> {
      ActionBinding actionBinding = new ActionBinding.Builder(arg.toString(), arg instanceof String ? getArgumentType((String)arg) : arg.getClass()).setIsLocal(true).setIsReturn(true).build();
      this.arguments.put(actionBinding.getName(), actionBinding);
    });
  }


  @Override
  public void doStep() {
    log.debug("[doStep] method entered.");

    // collect non-return args and find return arg to be filled
    List<ActionBinding> args = getArguments();
    List<Object> argBindings = new ArrayList();
    ActionBinding returnArg = null;
    boolean firstArg = true;
    if (args != null) {
      for (ActionBinding arg : args) {
        if (firstArg) {
          // first arg is ?actor -- leave out of THIS call
          firstArg = false;
        } else if (arg.isReturn) {
          if (returnArg != null) {
            log.error("More than one return value in executeAction for \"this\" method: " + this.cmd);
          }
          returnArg = arg;
        } else {
          if (!arg.isBound()) {
            log.error("Trying to call \"this\" method with unbound arguments for method: " + this.cmd);
          }
          Object value = arg.getBindingDeep();
          argBindings.add(value);
        }
      }
    }

    // find parent action context that "this" is referring to
    Context targetContext = this.caller;
    while (!targetContext.isAction()) {
      targetContext = targetContext.getParentContext();
    }

    // attempt to find method
    Method method = null;
    try {
      if (argBindings.isEmpty()) {
        method = StandardJavaMethods.getMethod(targetContext.getClass(), this.cmd);
      } else {
        method = StandardJavaMethods.getMethod(targetContext.getClass(), this.cmd, argBindings);
      }
    } catch (NoSuchMethodException e) {
      log.error("ThisContext method has no such method: {}", this.cmd);
    }

    // attempt to call method
    Object returnValue;
    try {
      if (method.isVarArgs()) {
        MethodHandle methodHandle = MethodHandles.lookup().bind(targetContext, this.cmd, MethodType.methodType(method.getReturnType(), method.getParameterTypes()));
        returnValue = methodHandle.invokeWithArguments(argBindings);
      } else if (argBindings.isEmpty()) {
        returnValue = method.invoke(targetContext);
      } else {
        returnValue = method.invoke(targetContext, argBindings);
      }

      // bind return value
      if (returnArg != null) {
        returnArg.bindDeep(returnValue);
        redistributeArguments();
      }
    } catch (Throwable e) {
      log.error("ThisContext threw an exception while calling method: {}", this.cmd, e);
    }

  }

  /**
   * Private constructor for copy method.
   *
   * @param caller
   * @param sm
   * @param cmd
   * @param arguments
   */
  private ThisContext(Context caller, StateMachine sm, String cmd, LinkedHashMap<String, ActionBinding> arguments) {
    super(caller, sm, cmd);
    copyArguments(arguments);
  }

  @Override
  public ThisContext copy(Context newParent) {
    ThisContext thisContext =  new ThisContext(newParent, newParent.stateMachine, cmd, this.arguments);
    copyInternal(thisContext);
    return thisContext;
  }
}
