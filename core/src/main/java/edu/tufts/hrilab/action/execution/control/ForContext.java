/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;

import java.util.List;

/**
 * This represents the semantics of the FOR control in an action script.
 * Provides a compact way to iterate over a range of (integer) values.
 *
 * @author luca
 *
 * Syntax:
 * for (!var = initial_value; !var compare_operator compare_value; !var increment) {
 * ....
 * }
 *
 * - !var is an int (doesn't have to be declared as a script-level variable, only exists within for context)
 * - initial_value (optional if !var already defined) is the initial value of !var (integer constant or variable)
 * - compare_operator is the comparison operator used to check for termination of the loop (one of lt, leq, gt, geq)
 * - compare_value is the value !var will be compared with (can be an integer constant or an integer variable)
 * - increment is an the operation carried out on !var after each loop (one of ++, --, +=N, -=N, *=N, /=N, where N is
 * an integer constant or an integer variable)
 * - the body of the loop is the sequence of script elements between for and endfor.
 *
 * Example: count from 0 to 10, printing at each loop.
 *
 * <pre>{@code
 * for (!i=0; !i lt 10; !i ++) {            // for(int i=0; i<=10; i++) {
 * op:log("info", "Countdown: !i");       //  loop body
 * }                                        // }
 *}</pre>
 *
 * EventSpec Steps:
 * for condition
 * block
 * step1
 * ...
 * stepN
 * endblock
 * endfor
 */

public class ForContext extends ArgumentBasedContext {
  protected String loopVarName = null;
  private CompOp compareOperator = null;
  private int compareValue;
  private String compareVar = null;
  private IncrOp incrementOperator = null;
  private int incrementAmount = 1; // default
  private String incrementAmountVar = null;
  private boolean firstIteration = true;
  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public ForContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "FOR");
    setupArguments(parser.getEventSpec().getInputArgs(), null);
    this.parser = parser;
  }

  /**
   * Prepare for the next iteration of the for loop. This includes cleaning up child contexts from
   * previous loop iterations.
   */
  private void initializeNextIteration() {
    // prepare for next loop iteration
    childContexts.clear(); // not strictly necessary, but prevents huge context trees from growing
    currParserStep = parser.getBody(); // prepare to re-parse, which happens in the doStep method
  }

  /**
   * Add the next child context to be executed, if one is available.
   */
  private void addNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  @Override
  protected void setupNextStep() {
    switch (getExecType()) {
      case SIMULATE_ACT:
      case ACT: {
        updateLoopData();
        if (shouldExecuteLoop()) {
          initializeNextIteration();
          addNextStep();
        }
      }
      break;
      default:
        // simulation -> get all the child contexts
        // tmf: how should execution handle number of loops? especially if condition args may not be bound
        addNextStep();
    }
  }

  /**
   * Update value of loop index.
   */
  private void updateLoopData() {
    // don't need to update loop data (e,g., increment/decrement/etc) on the first loop iteration
    if (firstIteration) {
      firstIteration = false;
      return;
    }

    ActionBinding loopVar = this.getArgument(loopVarName);
    int loopVarValue = (int) loopVar.getBinding();

    // If increment amount is a variable, get current value.
    if (incrementAmountVar != null) {
      incrementAmount = (int) this.getArgumentValue(incrementAmountVar);
    }

    // Update for loop variable
    switch (incrementOperator) {
      case INCR:
        loopVarValue += incrementAmount;
        break;
      case DECR:
        loopVarValue -= incrementAmount;
        break;
      case MULT:
        loopVarValue *= incrementAmount;
        break;
      case DIV:
        loopVarValue /= incrementAmount;
        break;
    }
    loopVar.bindDeep(loopVarValue);
  }

  /**
   * Check the loop criteria to determine if an iteration should be performed.
   * @return
   */
  private boolean shouldExecuteLoop() {
    ActionBinding loopVar = this.getArgument(loopVarName);
    int loopVarValue = (int) loopVar.getBinding();

    // If compare value is a variable, get current value.
    if (compareVar != null) {
      compareValue = (int) this.getArgumentValue(compareVar);
    }

    // Do comparison
    boolean shouldLoop = false;
    switch (compareOperator) {
      case LT:
        shouldLoop = loopVarValue < compareValue;
        break;
      case LEQ:
        shouldLoop = loopVarValue <= compareValue;
        break;
      case GEQ:
        shouldLoop = loopVarValue >= compareValue;
        break;
      case GT:
        shouldLoop = loopVarValue > compareValue;
        break;
    }
    return shouldLoop;
  }

  /**
   * Sets up the variable for the "for" context.
   *
   * @param inputArgs should be !var init_val compar_op compar_val incr_op (5 arguments)
   */
  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (inputArgs.size() >= 4) {
      loopVarName = inputArgs.get(0).toString();
      String initVal = (inputArgs.size() == 5) ? inputArgs.get(1).toString() : null;
      String comparOp = inputArgs.get((inputArgs.size() == 5) ? 2 : 1).toString();
      String comparVal = inputArgs.get((inputArgs.size() == 5) ? 3 : 2).toString();
      String incrOp = inputArgs.get((inputArgs.size() == 5) ? 4 : 3).toString();

      if (Utilities.isScriptVariable(loopVarName)) {
        ActionBinding variable = null;
        if (hasArgumentInScope(loopVarName)) {
          variable = this.getArgument(loopVarName);
        }

        if (variable == null) {
          if (Utilities.isLocalVariable(loopVarName) && initVal != null) {
            // Create local variable on the spot if necessary
            variable = new ActionBinding.Builder(loopVarName, Integer.class).setIsLocal(true).build();
            this.arguments.put(loopVarName, variable);
          } else {
            log.error("Variable " + loopVarName + " in FOR control element is not defined.");
            this.setStatus(ActionStatus.FAIL_SYNTAX);
            return;
          }
        }

        if (initVal != null) {
          if (Utilities.isScriptVariable(initVal)) {
            ActionBinding value = this.getArgument(initVal);

            if (value != null && Utilities.isIntegerType(value.getJavaType())) {
              variable.bind(value);
            } else {
              log.error("Initial value " + initVal + " not found or not integer type in FOR control element.");
              this.setStatus(ActionStatus.FAIL_SYNTAX);
              return;
            }
          } else if (Utilities.isInteger(initVal)) {
            variable.bind(initVal);
          } else {
            log.error("Initial value " + initVal + " with incorrect type or format in FOR control element.");
            this.setStatus(ActionStatus.FAIL_SYNTAX);
            return;
          }
        }

        this.compareOperator = Utilities.strToEnum(CompOp.class, comparOp);
        if (this.compareOperator == null) {
          log.error("Unsupported comparison operator " + comparOp + " in FOR control element.");
          this.setStatus(ActionStatus.FAIL_SYNTAX);
          return;
        }

        if (Utilities.isInteger(comparVal)) {
          this.compareValue = (int) Utilities.convertToType(int.class, comparVal);
        } else if (Utilities.isScriptVariable(comparVal) && this.getEvaluatedArgument(comparVal) != null) {
          this.compareVar = comparVal;
        } else {
          log.error("Compare value " + comparVal + " is not of integer type in FOR control element.");
          this.setStatus(ActionStatus.FAIL_SYNTAX);
          return;
        }

        if (incrOp.equals("++")) {
          this.incrementOperator = IncrOp.INCR;
        } else if (incrOp.equals("--")) {
          this.incrementOperator = IncrOp.DECR;
        } else if (incrOp.length() > 2) {
          switch (incrOp.substring(0, 2)) {
            case "+=":
              this.incrementOperator = IncrOp.INCR;
              break;
            case "-=":
              this.incrementOperator = IncrOp.DECR;
              break;
            case "*=":
              this.incrementOperator = IncrOp.MULT;
              break;
            case "/=":
              this.incrementOperator = IncrOp.DIV;
              break;
            default:
              log.error("Unexpected increment operator " + incrOp + " in FOR control element.");
              this.setStatus(ActionStatus.FAIL_SYNTAX);
              return;
          }
          String amount = incrOp.substring(2);
          if (Utilities.isInteger(amount)) {
            this.incrementAmount = (int) Utilities.convertToType(int.class, amount);
          } else if (Utilities.isScriptVariable(amount) && this.getEvaluatedArgument(amount) != null) {
            this.incrementAmountVar = amount;
          } else {
            log.error("Invalid increment amount " + amount + " in FOR control element.");
            this.setStatus(ActionStatus.FAIL_SYNTAX);
            return;
          }
        } else {
          log.error("Unexpected increment operator " + incrOp + " in FOR control element.");
          this.setStatus(ActionStatus.FAIL_SYNTAX);
          return;
        }
      } else {
        log.error("Expected variable as first argument in FOR control element.");
        this.setStatus(ActionStatus.FAIL_SYNTAX);
        return;
      }
    } else {
      log.error("Expected at least 4 arguments in FOR control element.");
      this.setStatus(ActionStatus.FAIL_SYNTAX);
      return;
    }
  }

  public enum CompOp {
    LT,
    LEQ,
    GEQ,
    GT
  }

  public enum IncrOp {
    INCR,
    DECR,
    MULT,
    DIV
  }

  public static boolean isIncrementOperator(String txt) {
    if (txt.length() >= 2) {
      return ((txt.equals("++") || txt.equals("--")) ||
              ((Utilities.isInteger(txt.substring(2)) || Utilities.isScriptVariable(txt.substring(2))) &&
                      (txt.startsWith("+=") || txt.startsWith("-=") || txt.startsWith("*=") || txt.startsWith("/="))));
    } else return false;
  }

  //FIXME: correctly set up args
  @Override
  public ForContext copy(Context newParent) {
    ForContext newFor = new ForContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newFor);
    newFor.loopVarName = loopVarName;
    newFor.compareOperator = compareOperator;
    newFor.compareValue = compareValue;
    newFor.compareVar = compareVar;
    newFor.incrementOperator = incrementOperator;
    newFor.incrementAmount = incrementAmount;
    newFor.incrementAmountVar = incrementAmountVar;
    return newFor;
  }
}
