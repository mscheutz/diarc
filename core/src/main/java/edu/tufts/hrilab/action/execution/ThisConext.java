package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.operators.StandardJavaMethods;
import edu.tufts.hrilab.action.state.StateMachine;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.reflect.Method;
import java.util.List;

public class ThisConext extends ArgumentBasedContext {
  public ThisConext(Context caller, StateMachine sm, String cmd, List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    super(caller, sm, cmd);
    setupArguments(inputArgs, returnArgs);

  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    inputArgs.forEach(arg -> {
      ActionBinding actionBinding = new ActionBinding.Builder(arg.toString(), arg.getClass()).setIsLocal(true).build();
      this.arguments.put(actionBinding.getName(), actionBinding);
    });
    returnArgs.forEach(arg -> {
      ActionBinding actionBinding = new ActionBinding.Builder(arg.toString(), arg.getClass()).setIsLocal(true).setIsReturn(true).build();
      this.arguments.put(actionBinding.getName(), actionBinding);
    });
  }


  @Override
  public void doStep() {
    log.debug("[doStep] method entered.");

    Method method = StandardJavaMethods.getMethod(this.caller.getClass(), this.cmd, this.get);

    if (method.isVarArgs()) {
      // TODO: EAK: understand why this doesn't work
//    MethodHandle methodHandle = MethodHandles.lookup().unreflect(method);
//    methodHandle = methodHandle.bindTo(o);
//    methodHandle.invokeWithArguments(args);

      MethodHandle methodHandle = MethodHandles.lookup().bind(o, name, MethodType.methodType(method.getReturnType(), method.getParameterTypes()));
      return methodHandle.invokeWithArguments(args);
    } else {
      return method.invoke(o, args);
    }
  }

  @Override
  public Context copy(Context newParent) {
    return null;
  }
}
