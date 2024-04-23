/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ControlFactory {

  protected final static Logger log = LoggerFactory.getLogger(ControlFactory.class);

  public static Context build(Context parent, StateMachine sm, ScriptParser parser) {
    Context controlContext = null;
    String cmd = parser.getEventSpec().getCommand();

    if (cmd == null) {
      log.error("Calling build with empty command.");
      return null;
    }

    Control c = Control.fromString(cmd);
    try {
      switch (c) {
        case ASYNC:
          controlContext = new AsynchronousContext(parent, sm, parser);
          break;
        case IF:
        case ELSEIF:
          log.debug("IF");
          controlContext = new IfContext(parent, sm, parser);
          break;
        case FOR:
          log.debug("FOR");
          controlContext = new ForContext(parent, sm, parser);
          break;
        case FOREACH:
          log.debug("FOREACH");
          controlContext = new ForeachContext(parent, sm, parser);
          break;
        case WHILE:
          log.debug("WHILE");
          controlContext = new WhileContext(parent, sm, parser);
          break;
        case THEN:
        case ELSE:
        case DO:
        case BLOCK:
          controlContext = new BlockContext(parent, sm, parser);
          break;
        case TRUE:
          controlContext = new TrueContext(parent, sm);
          if (parser.getBody() != null) {
            log.error("Syntax error: Unexpected argument to TRUE");
          }
          break;
        case AND:
          controlContext = new AndContext(parent, sm);
          ScriptParser andParser = parser.getBody();
          while (andParser != null) {
            controlContext.setupNextStep(andParser);
            andParser = andParser.getRest();
          }
          break;
        case OR:
          controlContext = new OrContext(parent, sm);
          ScriptParser orParser = parser.getBody();
          while (orParser != null) {
            controlContext.setupNextStep(orParser);
            orParser = orParser.getRest();
          }
          break;
        case NOT:
          log.debug("NOT");
          controlContext = new NotContext(parent, sm);
          ScriptParser notParser = parser.getBody();
          controlContext.setupNextStep(notParser);
          log.debug("eval: " + notParser.getCommand());
          break;
        case TRY:
          controlContext = new TryContext(parent, sm, parser);
          break;
        case CATCH:
          controlContext = new CatchContext(parent, sm, parser);
          break;
        case FINALLY:
          controlContext = new FinallyContext(parent, sm, parser);
          break;
        case EXIT:
          controlContext = new ExitContext(parent, sm, parser.getEventSpec().getInputArgs());
          break;
        case RETURN:
          controlContext = new ReturnContext(parent, sm);
          break;
        case JOIN:
          controlContext = new JoinContext(parent, sm, parser.getEventSpec().getInputArgs(), parser.getEventSpec().getReturnArgs());
          break;
        case OTHER:
          log.error("Unknown control element: " + cmd);
      }
    } catch (Exception e) {
      log.error("Encountered error while setting up control: " + cmd, e);
      return null;
    }
    return controlContext;
  }

  public enum Control {
    AND(true),
    ASYNC(true),
    BLOCK(true),
    CATCH(false),
    DO(false),
    ELSE(false),
    ELSEIF(false),
    END(false),
    ENDAND(false),
    ENDASYNC(false),
    ENDFOR(false),
    ENDIF(false),
    ENDNOT(false),
    ENDOR(false),
    ENDTRY(false),
    ENDWHILE(false),
    EXIT(true),
    FINALLY(false),
    FOR(true),
    FOREACH(true),
    IF(true),
    JOIN(true),
    NOT(true),
    OR(true),
    OTHER(false),
    RETURN(false),
    THEN(false),
    TRUE(true),
    TRY(true),
    WHILE(true);

    /**
     * If this control can start a control sequence (i.e. not and end... or something like else, do...).
     */
    private final boolean isStart;

    /**
     * Constructor.
     *
     * @param isStart if this control can start a control sequence
     */
    Control(boolean isStart) {
      this.isStart = isStart;
    }

    /**
     * Gets what control options can come next (END control comes first).
     *
     * NOTE: normally enum instance specific fields like this should go in the
     * constructor (e.g., the isStart field), but using other Control enums in
     * a Control enum constructor isn't allowed (i.e., illegal forward reference).
     *
     * @return list of controls
     */
    public List<Control> getNext() {
      switch (this) {
        case AND:
          return Collections.singletonList(ENDAND);
        case ASYNC:
          return Collections.singletonList(ENDASYNC);
        case BLOCK:
          return Collections.singletonList(END);
        case IF:
        case ELSEIF:
          return Collections.singletonList(THEN);
        case THEN:
          return Arrays.asList(ENDIF, ELSEIF, ELSE);
        case ELSE:
          return Collections.singletonList(ENDIF);
        case FOR:
        case FOREACH:
          return Collections.singletonList(ENDFOR);
        case NOT:
          return Collections.singletonList(ENDNOT);
        case OR:
          return Collections.singletonList(ENDOR);
        case TRY:
          return Arrays.asList(CATCH, FINALLY);
        case CATCH:
          return Arrays.asList(ENDTRY, CATCH, FINALLY);
        case WHILE:
          return Collections.singletonList(DO);
        case DO:
          return Collections.singletonList(ENDWHILE);
        default:
          return null;
      }
    }

    /**
     * @return true if this control can start a control sequence (i.e. not and end... or something like else, do...)
     */
    public boolean isStart() {
      return isStart;
    }

    /**
     * Case-insensitive version of valueOf. Returning OTHER instead of null.
     *
     * @param string
     * @return
     */
    public static Control fromString(String string) {
      Control c = Utilities.strToEnum(Control.class, string);
      if (c == null) {
        return OTHER;
      } else {
        return c;
      }
    }
  }
}
