package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.state.StateMachine;
import java.util.List;

public class NoInterruptContext extends ArgumentBasedContext {

    private final ScriptParser parser;
    private ScriptParser currParserStep;

    public NoInterruptContext(Context c, StateMachine sm, ScriptParser parser) {
        super(c, sm, "NOINTERRUPT");
        setupArguments(List.of(), List.of());
        this.parser = parser;
        init();
    }

    @Override
    protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
        // lexical scope only
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
    public Context copy(Context newParent) {
        NoInterruptContext newContext = new NoInterruptContext(newParent, newParent.getStateMachine(), parser);
        copyInternal(newContext);
        newContext.copyArguments(this.arguments);
        return newContext;
    }
}
