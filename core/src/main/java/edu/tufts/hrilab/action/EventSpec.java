/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.StringTokenizer;

/**
 * An event specification is a description of a step (event) in an action script.
 * Some events are realized as actions, and other events are controls (e.g. if, while).
 * The EventSpec class is designed to parse the String that describes the step and
 * provide simple accessors to the command (or control) and the list of arguments.
 *
 * This class is immutable.
 */
public final class EventSpec implements Serializable {

  private static final Logger log = LoggerFactory.getLogger(EventSpec.class);
  private final EventType type;
  private String actor = null;
  private String command = null;
  private final List<String> inputArgs = new ArrayList<>();
  private final List<String> returnArgs = new ArrayList<>();

  public static class Builder {
    private EventType type;
    private String actor = null;
    private String command = null;
    private List<String> inputArgs = new ArrayList<>();
    private List<String> returnArgs = new ArrayList<>();

    public Builder(EventType type) {
      this.type = type;
    }

    public EventSpec build() {
      return new EventSpec(type, actor, command, inputArgs, returnArgs);
    }

    public Builder setActor(String actor) {
      this.actor = actor;
      return this;
    }

    public Builder setCommand(String command) {
      this.command = command;
      return this;
    }

    public Builder setInputArgs(List<String> inputArgs) {
      this.inputArgs = new ArrayList<>(inputArgs);
      return this;
    }

    public Builder addInputArg(String inputArg) {
      this.inputArgs.add(inputArg);
      return this;
    }

    public Builder setReturnArgs(List<String> returnArgs) {
      this.returnArgs = new ArrayList<>(returnArgs);
      return this;
    }

    public Builder addReturnArg(String returnArg) {
      this.returnArgs.add(returnArg);
      return this;
    }

  }

  /**
   * Constructor used by the Builder. Private to enforce Builder use (once the other constructors are removed).
   *
   * @param eventType
   * @param actor
   * @param command
   * @param inputArgs
   * @param returnArgs
   */
  private EventSpec(EventType eventType, String actor, String command, List<String> inputArgs, List<String> returnArgs) {
    this.type = eventType;
    this.actor = (actor != null) ? actor : "?actor";
    this.command = command;
    this.inputArgs.addAll(inputArgs);
    this.returnArgs.addAll(returnArgs);
  }

  /**
   * Use the Builder or constructor that takes in a Predicate.
   * @param eventType
   * @param eventString
   */
  @Deprecated
  public EventSpec(EventType eventType, String eventString) {
    type = eventType;

    if (eventString == null || eventString.isEmpty()) {
      log.error("EventString is null or empty.");
      actor = null;
      command = null;
    } else {

      // check if space delimited or functional syntax, and parse event string accordingly
      eventString = eventString.trim();
      StringTokenizer esTokenizer = new StringTokenizer(eventString);
      String firstToken = esTokenizer.nextToken();
      if (firstToken.contains("(")) {
        // functional syntax (e.g., "walk(forward)" or "dempster.walk(forward)")
        parseFunctionalEventSpec(eventString);
      } else {
        //TODO:brad: do we care about supporting the space delimited event spec syntax?
        // space delimited syntax (e.g., "walk forward" or "dempster.walk forward")
        parseSpaceDelimitedEventSpec(eventString);
      }

      // checks
      if (Utilities.isScriptVariable(command) && type == EventType.GOAL) {
        if (esTokenizer.hasMoreTokens()) {
          throw new IllegalArgumentException("Invalid event spec. Passing additional args with a variable is not currently supported: " + eventString);
        }
      }
    }

  }

  /**
   * Build ACTION or GOAL EventSpec from Predicate representation (i.e., action(actor,args)).
   *
   * TODO: generalize this method to work for non-ACTION types, using the same semantics that Goal expects.
   *
   * @param eventPredicate
   */
  public EventSpec(Predicate eventPredicate) {
    if (eventPredicate.size() > 0) {
      this.actor = eventPredicate.get(0).toString();
      if (eventPredicate.getName().equals("goal")) {
        type = EventType.GOAL;
        Predicate state = (Predicate) eventPredicate.get(1);
        command = state.getName();
        for (Symbol arg : state.getArgs().subList(1, eventPredicate.getArgs().size())) {
          inputArgs.add(arg.toString());
        }
      } else {
        command = eventPredicate.getName();
        type = EventType.ACTION;
        for (Symbol arg : eventPredicate.getArgs().subList(1, eventPredicate.getArgs().size())) {
          inputArgs.add(arg.toString());
        }
      }
    } else {
      log.error("[EventSpec] trying to create event spec with invalid form: " + eventPredicate);
      type = null;
    }
  }

  private void parseFunctionalEventSpec(String eventString) {
    int oParIndex = eventString.indexOf("(");
    int cParIndex = eventString.lastIndexOf(")");

    // do some parentheses checking
    if (oParIndex == -1 || eventString.length() - 1 != cParIndex) {
      throw new IllegalArgumentException("Invalid event spec. Perhaps the parentheses are mismatched: " + eventString);
    }

    // parse command and actor
    String firstToken = eventString.substring(0, oParIndex);
    parseCommandAndActor(firstToken, eventString);

    String remainingEventString = eventString.substring(oParIndex + 1, cParIndex).replaceAll(" ", "");
    inputArgs.addAll(Util.tokenizeArgs(remainingEventString));
  }

  //TODO:brad: why are we still supporting this? especially given that we use ASL now
  private void parseSpaceDelimitedEventSpec(String eventString) {
    StringTokenizer esTokenizer = new StringTokenizer(eventString);
    String token;
    String subToken;

    String firstToken = esTokenizer.nextToken();
    parseCommandAndActor(firstToken, eventString);

    while (esTokenizer.hasMoreTokens()) {
      token = esTokenizer.nextToken();
      if (token.startsWith("\"")) {
        // deal with strings that have spaces
        StringBuilder tokensb = new StringBuilder(token);
        if (!token.endsWith("\"") || token.length() == 1) {
          while (esTokenizer.hasMoreTokens()) {
            subToken = esTokenizer.nextToken();
            tokensb.append(" ").append(subToken);
            if (subToken.endsWith("\"")) {
              break;
            }
          }
        }
        token = tokensb.toString();
      }
      inputArgs.add(token);
    }
  }

  private void parseCommandAndActor(String token, String eventString) {

    if (token.contains(".")) {
      if (type == EventType.OPERATOR || type == EventType.CONTROL) {
        throw new IllegalArgumentException("Invalid event spec. Setting the actor with the dot notation in " + type + " is not currently supported: " + eventString);
      }

      String[] parts = token.split("\\.");
      actor = parts[0];
      command = parts[1];
    } else {
      actor = "?actor";
      command = token;
    }
  }

  /**
   * Get EventType.
   *
   * @return type
   */
  public EventType getType() {
    return type;
  }

  public Predicate getPredicateForm() {
    switch (type) {
      case ACTION: {
        // command(actor,inputArgs)
        List<String> tmpArgs = new ArrayList<>();
        tmpArgs.add(actor);
        tmpArgs.addAll(inputArgs);
        return Factory.createPredicate(command, tmpArgs.toArray(new String[0]));
      }
      case GOAL: {
        // goal(actor,command(inputArgs))
        return Factory.createPredicate("goal", Factory.createSymbol(actor), Factory.createPredicate(command, inputArgs.toArray(new String[0])));
      }
      case OBSERVATION: {
        // obs(actor,command(inputArgs))
        List<String> tmpArgs = new ArrayList<>();
        tmpArgs.add(actor);
        tmpArgs.addAll(inputArgs);
        return Factory.createPredicate("obs", Factory.createSymbol(actor), Factory.createPredicate(command, inputArgs.toArray(new String[0])));
      }
      case TSC:
      case OPERATOR:
      case CONTROL: {
        // command(inputArgs)
        return Factory.createPredicate(command, inputArgs.toArray(new String[0]));
      }
      default:
        return null;
    }
  }

  /**
   * Get command.
   *
   * @return command
   */
  public String getCommand() {
    return command;
  }

  public String getActor() {
    return actor;
  }

  /**
   * Get input arguments.
   *
   * @return shallow copy of arguments
   */
  public List<String> getInputArgs() {
    return new ArrayList<>(inputArgs);
  }

  /**
   * Get return arguments.
   *
   * @return shallow copy of arguments
   */
  public List<String> getReturnArgs() {
    return new ArrayList<>(returnArgs);
  }

  /**
   * Get all arguments. Input args followed by return args.
   *
   * @return shallow copy of arguments
   */
  public List<String> getAllArgs() {
    List<String> allArgs = new ArrayList<>(inputArgs);
    allArgs.addAll(returnArgs);
    return allArgs;
  }

  @Override
  public String toString() {
    StringBuilder ret_val = new StringBuilder();
    if (actor != null && actor.length() > 0) {
      ret_val.append(actor).append(".");
    }
    ret_val.append(command);
    for (String s : inputArgs) {
      ret_val.append(" ").append(s);
    }
    return ret_val.toString();
  }

  public enum EventType {
    ACTION,
    OBSERVATION,
    CONTROL,
    OPERATOR,
    GOAL,
    TSC;

    public static EventType fromString(String eventStr) {
      if (eventStr != null) {
        for (EventType e : EventType.values()) {
          if (eventStr.equalsIgnoreCase(e.name())) {
            return e;
          }
        }
      }

      // temporarily support legacy types
      switch (eventStr) {
        case "actspec":
          return ACTION;
        case "opspec":
          return OPERATOR;
        case "statespec":
          return GOAL;
        case "obspec":
          return OBSERVATION;
      }

      log.warn("Unknown event type: " + eventStr);
      return null;
    }

  }

  @Override
  public boolean equals(Object o) {
    if (!o.getClass().equals(this.getClass())) {
      return false;
    }
    EventSpec other = (EventSpec) o;
    if (!Objects.equals(other.command, this.command)) {
      return false;
    }
    if (!Objects.equals(other.type, this.type)) {
      return false;
    }
    if (!Objects.equals(other.actor, this.actor)) {
      return false;
    }
    if (!Objects.equals(other.inputArgs, this.inputArgs)) {
      return false;
    }
    return true;
  }

  @Override
  public int hashCode() {
    int result = 1;
    result = 31 * result + Objects.hashCode(command);
    result = 31 * result + Objects.hashCode(type);
    result = 31 * result + Objects.hashCode(actor);
    return 31 * result + Objects.hashCode(inputArgs);
  }
}
