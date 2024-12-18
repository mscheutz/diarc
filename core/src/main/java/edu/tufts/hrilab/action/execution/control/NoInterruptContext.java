package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.state.StateMachine;

public class NoInterruptContext extends Context {

    private final ScriptParser parser;
    private ScriptParser currParserStep;

    public NoInterruptContext(Context c, StateMachine sm, ScriptParser parser) {
        super(c, sm, "NOINTERRUPT");
        this.parser = parser;
        init();
    }

    private void init() {
        currParserStep = parser.getBody();
    }

    @Override
    public void resetConcreteContext() {
        init();
    }

    @Override
    protected void setupNextStep() {
        if (currParserStep != null) {
            setupNextStep(currParserStep);
            currParserStep = currParserStep.getRest();
        }
    }

    @Override
    public boolean isInterruptible() {
        return false;
    }

    @Override
    public Context copy(Context newParent) {
        NoInterruptContext newContext = new NoInterruptContext(newParent, newParent.getStateMachine(), parser);
        copyInternal(newContext);
        return newContext;
    }
}
