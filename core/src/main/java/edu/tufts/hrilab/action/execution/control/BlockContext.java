/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import java.util.List;

/**
 *
 * @author willie
 */
public class BlockContext extends ArgumentBasedContext {
  private ScriptParser parser;
  private ScriptParser currParserStep;

  public BlockContext(Context c, StateMachine sm, ScriptParser  parser) {
    super(c, sm, "BLOCK");
    setupArguments(List.of(), List.of());
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    // lexical scope only; no declared parameters
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
  public BlockContext copy(Context newParent) {
    BlockContext newBlock = new BlockContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newBlock);
    newBlock.copyArguments(this.arguments);
    return newBlock;
  }

}
