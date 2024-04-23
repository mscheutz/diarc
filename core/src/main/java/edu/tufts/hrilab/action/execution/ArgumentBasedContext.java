/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public abstract class ArgumentBasedContext extends Context {

  /**
   * Arguments defined in this context.
   */
  protected final LinkedHashMap<String, ActionBinding> arguments = new LinkedHashMap<>();

  protected static Pattern variable = Pattern.compile("[^\\\\]([?!]\\w+)");

  protected ArgumentBasedContext(Context caller, StateMachine sm, String cmd) {
    super(caller, sm, cmd);
  }

  protected ArgumentBasedContext(Context caller, StateMachine sm, String cmd, ExecutionType executionType) {
    super(caller, sm, cmd, executionType);
  }

  abstract protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs);

  /**
   * Create predicate from the context cmd and arguments: "cmd(arg0, arg1, ...)".
   *
   * @return
   */
  public Predicate getSignatureInPredicateForm() {
    List<Symbol> args = new ArrayList<>();
    if (arguments != null) { // this check is necessary because when this class is instantiated, the context class initializes the status and calls this method
      for (ActionBinding arg : arguments.values()) {
        if (arg.isReturn() || arg.isLocal) {
          // return args are at the end and we don't want them here, so we're done
          break;
        } else if (arg.isBound()) {
          Symbol val = Utilities.createFOL(arg);
          if (val == null) {
            args.add(Factory.createSymbol("null"));
          } else {
            args.add(val);
          }
        } else {
          args.add(Factory.createSymbol(arg.name));
        }
      }
    }
    return new Predicate(this.cmd, args);
  }

  /**
   * Get list of copied and evaluated arguments in order. ?actor role is alwyas first.
   * @return
   */
  @Override
  public List<ActionBinding> getArguments() {
    List<ActionBinding> args = new ArrayList<>();

    for (ActionBinding contextArg : arguments.values()) {
      args.add(getEvaluatedArgument(contextArg.name));
    }

    return args;
  }

  /**
   * Check if this context explicitly has the specified argument. This does not search the entire
   * action scope.
   *
   * @param bname argument name
   * @return true if argument exists
   */
  public boolean hasLocalArgument(String bname) {
    return (arguments.get(bname) != null);
  }

  /**
   * Check if the specified argument exists in action scope. Scope in the context tree is
   * defined as within an ActionContext (i.e., will not search outside of the containing
   * ActionContextzs).
   *
   * @param bname
   * @return
   */
  public boolean hasArgumentInScope(String bname) {
    return (getArgumentSilent(bname) != null);
  }

  /**
   * Retrieve the (deep) value of an argument, evaluating any unevaluated
   * arguments in the process. Recurses up the caller tree to find argument, if
   * it exists.
   *
   * @param bname the argument's name
   * @return the deep binding associated with that name in the current context
   */
  @Override
  public Object getArgumentValue(String bname) {
    ActionBinding argument = getEvaluatedArgument(bname);
    if (argument == null) {
      return null;
    }

    return argument.getBindingDeep();
  }

  /**
   * Return an ActionBinding with the (deep) value of an argument. This will
   * attempt to evaluate any unevaluated arguments (e.g., strings containing free variables), and
   * return them in a (potentially cloned) ActionBinding instance. Recurses up
   * the caller tree to find argument, if it does not exist in this context.
   *
   * @param bname the argument's name
   * @return an (potentially cloned) ActionBinding associated with the bname in
   * the current context
   */
  @Override
  protected ActionBinding getEvaluatedArgument(String bname) {
    ActionBinding argument = getArgumentSilent(bname);

    if (argument == null) {
      log.error(this + " [getEvaluatedArgument] cannot find argument in this scope: " + bname);
      return null;
    }

    // "evaluate" any unevaluated variables
    if (argument.isReturn) {
      // do nothing -- keep this potentially unbound return argument as it is
    } else if (argument.isBound()) {
      // for bound args containing free variables, attempt to bind the free variables.
      Object bindingDeep = argument.getBindingDeep();
      if (bindingDeep instanceof String) {
        String bindingDeepString = (String) bindingDeep;
        if (bindingDeepString.equalsIgnoreCase("null")) {
          // TODO: do we want to do this?
          // turn "null" String into null Object.
          ActionBinding bound = argument.clone(); // cloning here to not permanently change underlying binding
          bound.bind(null);
          argument = bound;
        } else if (variable.matcher(" " + bindingDeepString).find()) {
          // TODO: the " " + inside the variable matcher is bc the regex is broken without it. fix this...
          Object value = bindText(bname, bindingDeepString);
          ActionBinding bound = argument.clone(); // cloning here to not permanently bind free variables to values that could change during execution
          bound.bind(value);
          argument = bound;
        }
      }
    }

    return argument;
  }

  /**
   * Retrieve the type of an argument
   *
   * @param bname the argument's name
   * @return the type associated with that name in the current context
   */
  @Override
  public Class<?> getArgumentType(String bname) {
    Class<?> clazz = null;

    // first check if bname is an argument name
    ActionBinding argument = getArgument(bname);
    if (argument != null) {
      clazz = argument.getBindingTypeDeep();
    }

    // must be an argument value, try to infer the type
    if (clazz == null) {
      clazz = Utilities.getArgumentType(bname);
    }

    return clazz;
  }

  /**
   * Finds the ActionBinding in this or parent contexts. Returns null if no binding with
   * that name exists. Similar to getEvaluatedArgument but does not attempt to evaluate or modify the
   * ActionBinding in an way.
   *
   * @param bname Binding name
   * @return the closest binding with that name in the context tree
   */
  @Override
  protected ActionBinding getArgument(String bname) {
    ActionBinding target = getArgumentSilent(bname);

    if (target == null && Utilities.isScriptVariable(bname)) {
      // FIXME: why does this throw an error if it can return a null value?
      //        observers take in a term representing the condition to observe which may be different from
      //        the variable in the predicate they are trying to observe
      log.error(this + " [getArgument] cannot find argument in this scope: " + bname);
    }
    return target;
  }

  @Override
  protected ActionBinding getArgumentSilent(String bname) {
    ActionBinding target = arguments.get(bname);
    if (target == null) {
      if (caller != null) {
        // to enforce proper variable scoping, don't recurse into parent actions
        if (caller.isAction()) {
          target = caller.getLocalArgument(bname);
        } else {
          target = caller.getArgumentSilent(bname);
        }
      }
    }
    return target;
  }

  @Override
  protected ActionBinding getLocalArgument(String bname) {
    return arguments.get(bname);
  }

  /**
   * Copy arguments into this Context. This is currently only used to copy a Context.
   * This first clears any existing arguments and then performs a deep copy on the passed in
   * arguments.
   *
   * @param argumentsToCopy
   */
  protected void copyArguments(LinkedHashMap<String, ActionBinding> argumentsToCopy) {
    this.arguments.clear();

    for (ActionBinding arg : argumentsToCopy.values()) {
      Object value = arg.getBinding();
      // passing an ActionBinding from the source context tree will result in an un-clean copy, essentially creating a
      // reference to an ActionBinding in the source context from the copied context. Instead, we want to create
      // a reference to the ActionBinding in the new parent context by using the ActionBinding's name which will properly
      // find the correct reference in the addArgument method.
      if (value instanceof ActionBinding) {
        value = ((ActionBinding) value).getName();
      }
      addArgument(arg.getName(), arg.getJavaType(), arg.getSemanticType(), value, arg.getDefaultValue(), arg.isLocal(), arg.isReturn(), arg.isVarArg());
    }
  }

  /**
   * Adds a new argument to the current context. This is convenience method to the more general addArgument.
   * This method sets the following values for fields that are not exposed:
   *   semanticType : null
   *   isVarArgs : false
   *   defaultValue : null
   *
   * @param bName the name of the argument
   * @param javaType the java type of the argument
   * @param bValue the value of the argument
   * @param bLocal whether the argument is a local script variable or not
   * @param bReturn whether the argument is a return variable or not
   * @return true if argument was added to context, false if it conflicts with
   * existing argument with same name
   */
  protected boolean addArgument(String bName, Class<?> javaType, Object bValue, boolean bLocal, boolean bReturn) {
    return addArgument(bName, javaType, null, bValue, null, bLocal, bReturn, false);
  }

  /**
   * Adds a new argument to the current context
   *
   * @param bName the name of the argument
   * @param javaType the java type of the argument
   * @param semanticType the semantic type of the argument
   * @param bValue the value of the argument
   * @param bDefault the default value of the argument
   * @param bLocal whether the argument is a local script variable or not
   * @param bReturn whether the argument is a return variable or not
   * @param bIsVarArg whether the argument is a varArg argument ()
   * @return true if argument was added to context, false if it conflicts with
   * existing argument with same name
   */
  protected boolean addArgument(String bName, Class<?> javaType, String semanticType, Object bValue, String bDefault, boolean bLocal, boolean bReturn, boolean bIsVarArg) {
    ActionBinding newArg;
    if (bValue == null) {
      newArg = new ActionBinding.Builder(bName, javaType).setSemanticType(semanticType)
              .setDefaultValue(bDefault).setValue(null)
              .setIsLocal(bLocal).setIsReturn(bReturn).setIsVarArg(bIsVarArg)
              .build();
    } else {
      String valueString = bValue.toString();
      if (valueString.length() > 0 && !valueString.contains(" ") && (valueString.startsWith("?") || valueString.startsWith("!"))) {
        // spec/bValue is a variable
        // find closest parent context in scope in which it is defined and use that ActionBinding
        // in the case that a default value has been defined, it's ok to not provide an explicit value
        ActionBinding parentArg = null;
        if (hasArgumentInScope(valueString)) {
          parentArg = getArgument(valueString);
        } else if (bDefault == null && bName.startsWith("?")) {
          // no argument in scope and no default value
          log.error(this + " No default value or passed in value found for argument: " + valueString);
        }

        if (caller != null && caller.isAsynchronous()) {
          // copy value of references (i.e., ActionBindings pointing to ActionBindings)
          // this is necessary to prevent references from changing values during execution (e.g., if the
          // async is inside a loop and an arg changes value during subsequent iterations)
          newArg = new ActionBinding.Builder(bName, javaType).setSemanticType(semanticType)
                  .setDefaultValue(bDefault).setValue(parentArg.getBindingDeep())
                  .setIsLocal(bLocal).setIsReturn(bReturn).setIsVarArg(bIsVarArg)
                  .build();
        } else {
          newArg = new ActionBinding.Builder(bName, javaType).setSemanticType(semanticType)
                  .setDefaultValue(bDefault).setValue(parentArg)
                  .setIsLocal(bLocal).setIsReturn(bReturn).setIsVarArg(bIsVarArg)
                  .build();
        }
      } else if (bValue instanceof String && variable.matcher(" " + valueString).find()) {
        // TODO: the " " + inside the variable matcher is bc the regex is broken without it. fix this...
        String value = bindText(bName, valueString);
        newArg = new ActionBinding.Builder(bName, javaType).setSemanticType(semanticType)
                .setDefaultValue(bDefault).setValue(value)
                .setIsLocal(bLocal).setIsReturn(bReturn).setIsVarArg(bIsVarArg).build();

      } else {
        // else spec itself is a value, create a bound ActionBinding
        newArg = new ActionBinding.Builder(bName, javaType).setSemanticType(semanticType)
                .setDefaultValue(bDefault).setValue(bValue)
                .setIsLocal(bLocal).setIsReturn(bReturn).setIsVarArg(bIsVarArg).build();
      }
    }

    if (!arguments.containsKey(bName)) {
      log.debug("adding " + bName + ":" + javaType + " with value " + bValue);
      arguments.put(bName, newArg);
      return true;
    } else {
      log.error(this + " An argument with the same name (" + bName + ") already exists.");
      return false;
    }
  }

  /**
   * Specify the (deep) value of an argument
   *
   * @param bname the argument's name
   * @param val the value to which the argument should be bound
   */
  protected void setArgument(String bname, Object val) {
    ActionBinding target = arguments.get(bname);

    if (target != null) {
      target.bindDeep(val);
    } else {
      log.error(this + " Argument not in this context: " + bname + ". Arguments: " + arguments);
    }
  }

  /**
   * Given the original Predicate, for each argument that is an unbound
   * variable, create a binding and put it in a new Predicate.
   *
   * @param orig
   * @return New Predicate with unbound arguments bound, if possible.
   */
  public Predicate bindPredicate(Symbol orig) {
    Predicate boundPredicate = null;

    if (orig.isVariable() || Utilities.isScriptVariable(orig.getName())) {
      // isScriptVariable(orig.getName()) is for cases like goal:?pred which creates a goal predicate ?pred()
      String name = orig.getName();
      Object val = getArgumentValue(name);
      if (val != null && Term.class.isAssignableFrom(val.getClass())) {
        // val is already in the fol class hierarchy
        boundPredicate = new Predicate((Term)val);
      } else {
        log.error(this + " [bindPredicate] stand-alone free-variables must be of type Predicate: " + orig);
      }
    } else if (orig.isTerm()) {
      Term orig_term = (Term) orig;
      List<Symbol> boundTerms = new ArrayList<>();
      for (Symbol arg : orig_term.getArgs()) {
        if (Utilities.isScriptVariable(arg)) {
          Symbol boundArg = Utilities.createFOL(getEvaluatedArgument(arg.getName()));
          if (boundArg == null) {
            boundTerms.add(arg);
          } else {
            boundTerms.add(boundArg);
          }
        } else if (arg.hasArgs()) {
          boundTerms.add(bindPredicate(arg));
        } else {
          boundTerms.add(arg);
        }
      }
      boundPredicate = new Predicate(orig.getName(), boundTerms);
    } else {
      log.error(this + " [bindPredicate] cannot bind Symbol: " + orig);
    }

    return boundPredicate;
  }

  protected void redistributeArguments() {
    for (ActionBinding arg : arguments.values()) {
      if (arg.isReturn) {
        ActionBinding parentBinding = caller.getArgument(arg.name);
        Object returnArgValue = arg.getBindingDeep();
        if (parentBinding != null && parentBinding.getJavaType() == arg.getJavaType()) {
          parentBinding.bindDeep(returnArgValue);
        }

        // Update logical value of execution context if boolean or justification return value is available,
        // otherwise logicalValue will keep its initial value of true
        //
        // returnArgValue should never be null, otherwise it will throw a null pointer exception
        // when verifying return value. At the moment the return value is null during simulation
        // and is being set only for boolean and justifications
        if (returnArgValue != null) {
          if (returnArgValue instanceof Boolean) {
            setLogicalValue((boolean) returnArgValue);
          } else if (returnArgValue instanceof Justification) {
            setLogicalValue(((Justification) returnArgValue).getValue());
          }
        } else {
          if (arg.getJavaType().equals(Boolean.class)) {
            arg.bindDeep(true);
          } else if (arg.getJavaType().equals(Justification.class)) {
            arg.bindDeep(new ConditionJustification(true));
          }
        }
      }
    }
  }

  /**
   * Parses text and replaces args with their values.
   *
   * @param inText text to parse
   * @return text with args replaced by their values
   */
  protected String bindText(String bName, String inText) {
    inText = " " + inText; // Avoid issue with regex TODO: Actually write a better regex...
    StringBuilder outText = new StringBuilder();
    Matcher m = variable.matcher(inText); // Add space at beginning for regex purposes...
    String tmpToken = null, token = null;
    Object tokenObj = null;
    int start = 0; // start of most recent match
    int end = 0;   // end of most recent match

    while (m.find()) {
      start = m.start(1);     // Position of arg (capturing group) in string inText
      tmpToken = m.group(1);  // Arg name (string inside capturing group, e.g. "?var")
      outText.append(inText.substring(end, start)); // Keep text before arg

      // Get the argument
      if (bName.equals(tmpToken)) {
        // if binding name is contained in binding value, start look-up one
        // step higher in the call tree to avoid infinite loop. This happens
        // when a script passes an arg to a sub-script that wraps the arg
        // in JS and assigns it to an arg with the same arg name.
        tokenObj = caller.getArgumentValue(tmpToken);
      } else {
        tokenObj = getArgumentValue(tmpToken);
      }
      if (tokenObj == null) {
        log.trace(this + ": unable to find " + tmpToken);
        tokenObj = tmpToken;
      }
      token = tokenObj.toString();
      // token might be bound to another string, which needs to be
      // checked again
      if (token.startsWith("\"")) {
        token = bindText(tmpToken, token);
      }
      outText.append(token);
      end = m.end(1);
    }
    outText.append(inText.substring(end, inText.length()));
    return outText.substring(1); // Remove space at beginning
  }

}
