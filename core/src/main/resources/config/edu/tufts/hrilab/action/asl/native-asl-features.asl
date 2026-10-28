/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
import java.lang.String;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.execution.AslTask;
import edu.tufts.hrilab.action.asl.CompiledAction;

(String ?summary) = exampleNativeAslFeatures() {
  long !input = 21;
  long !answer = 0;
  AslTask !task;
  ActionStatus !status;
  CompiledAction !module;

  !task = async(!answer) {
    !module = op:compileAsl("example-runtime.asl",
        "(long ?result) = doubleValue(long ?input) { ?result = ?input * 2; }",
        "doubleValue");
    !answer = invoke(!module, ?actor, !input);
  }
  (!status, !answer) = join(!task);
  if (!answer == 42) {
    ?summary = "answer=" + !answer;
  } else {
    exit(FAIL);
  }
}
