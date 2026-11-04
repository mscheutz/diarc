/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

/**
 *
 * @author willie
 */
public class BlockContext extends Context {
  private ScriptParser parser;
  private ScriptParser currParserStep;

  public BlockContext(Context c, StateMachine sm, ScriptParser  parser) {
    super(c, sm, "BLOCK");
    this.parser = parser;
    this.currParserStep = parser.getBody();
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
    return newBlock;
  }

}
