/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class is mostly to help with finding the relevant parts of a
 * control block (e.g., while, if/else, for, etc).
 *
 * NOTE: now this class is only needed for CONTROL types and could probably be
 * removed if the ActionScriptLanguageParser class and EventSpec classes handled grouping the bodies
 * of controls into single EventSpec instances. This would require EventSpecs to optionally contain children
 * EventSpecs, but the ASLParser should be able to easily handle this, since it's already parsing the actual ASL files.
 *
 * @author willie
 */
public class ScriptParser {
  private static final Logger log = LoggerFactory.getLogger(ScriptParser.class);
  private LinkedList<EventSpec> script;
  private EventSpec event;
  private String cmd;
  private ScriptParser rest;
  private ScriptParser body;
  private boolean parsed = false;

  public ScriptParser(List<EventSpec> script) {
    this.script = new LinkedList<>(script);
  }

  public boolean isEmpty() {
    return script.isEmpty();
  }

  /**
   * Get the EventType of the next EventSpec.
   *
   * @return
   */
  public EventSpec.EventType getType() {
    if (!parsed) {
      parse();
    }
    if (event != null) {
      return event.getType();
    } else return null;
  }

  /**
   * Returns the next EventSpec.
   *
   * @return
   */
  public EventSpec getEventSpec() {
    if (!parsed) {
      parse();
    }
    return event;
  }

  /**
   * The command in String form. Mostly just useful for debugging.
   *
   * @return
   */
  public String getCommand() {
    if (!parsed) {
      parse();
    }
    return cmd;
  }

  /**
   * In the case of a control EventSpec, this returns the body the of control.
   * For example, for While control, the body consists of the condition, and all
   * the contents in the "do" block.
   *
   * @return
   */
  public ScriptParser getBody() {
    if (!parsed) {
      parse();
    }
    return body;
  }

  public ScriptParser getRest() {
    if (!parsed) {
      parse();
    }
    return rest;
  }

  private void parse() {
    if (log.isDebugEnabled()) {
      log.debug("Parsing... " + script + ";");
    }
    if (script.isEmpty()) {
      parsed = true;
      return;
    }

    // create copy to parse
    LinkedList<EventSpec> scriptCopy = new LinkedList<>(script);
    event = scriptCopy.poll();
    EventSpec.EventType type = event.getType();
    cmd = event.getCommand();

    if (type == EventSpec.EventType.CONTROL) {
      List<String> nextControl = lookupNextControl(cmd);
      if (nextControl == null) {
        // control is keyword, e.g. TRUE
        body = null;
      } else {
        EventSpec next = scriptCopy.poll();
        int nested = 0;
        LinkedList<EventSpec> controlBody = new LinkedList<>();
        while (next != null && (!nextControl.contains(next.getCommand()) || nested > 0)) {
          nested += updateNested(next, cmd);
          controlBody.add(next);
          next = scriptCopy.poll();
        }
        body = new ScriptParser(controlBody);
        if (next != null && !next.getCommand().startsWith("end")) {
          scriptCopy.push(next); // Keep "non-end" controls in the list to process them next time.
        }
      }
    }

    // no extra work needed for non-control types

    if (!scriptCopy.isEmpty()) {
      rest = new ScriptParser(scriptCopy);
    }
    parsed = true;
  }

  /**
   * For a given control, look up the list of expected next controls.
   *
   * @param cmd command
   * @return list of next controls for this commands
   */
  private List<String> lookupNextControl(String cmd) {
    switch (cmd) {
      case "block":
        return Collections.singletonList("endblock");
      case "async":
        return Collections.singletonList("endasync");
      case "if":
        return Collections.singletonList("endif");
      case "then":
        return Arrays.asList("elseif", "else");
      case "elseif":
      case "else":
      case "do":
        return Collections.emptyList();
      case "while":
        return Collections.singletonList("endwhile");
      case "for":
      case "foreach":
        return Collections.singletonList("endfor");
      case "not":
        return Collections.singletonList("endnot");
      case "and":
        return Collections.singletonList("endand");
      case "or":
        return Collections.singletonList("endor");
      case "try":
        return Collections.singletonList("endtry");
      case "catch":
        return Arrays.asList("catch", "finally");
      case "finally":
        return Collections.emptyList();
      default:
        return null;  // Null is different from an empty list, look at parse().
    }
  }

  /**
   * Helper method to get the "base" control for a specific command.
   * For example, the base control of "then" or "else" is "if".
   *
   * @param cmd command
   * @return base control
   */
  private String baseControl(String cmd) {
    switch (cmd) {
      case "if":
      case "then":
      case "elseif":
      case "else":
        return "if";
      case "while":
      case "do":
        return "while";
      default:
        return cmd;
    }
  }

  /**
   * Updates the nested control count.
   *
   * @param next next control
   * @param cmd  current command
   * @return increment/decrement
   */
  private int updateNested(EventSpec next, String cmd) {
    if (next.getCommand().equals(baseControl(cmd))) {
      return +1; // We're dealing with a control of the same type. Increment nested control count.
    }
    List<String> ends = lookupNextControl(baseControl(cmd)); // Get end(s) for the base control of cmd.
    if (ends != null && ends.contains(next.getCommand())) {
      return -1; // We're dealing with the end of a control of the same type. Decrement nested control count.
    }
    return 0; // No change
  }
}
