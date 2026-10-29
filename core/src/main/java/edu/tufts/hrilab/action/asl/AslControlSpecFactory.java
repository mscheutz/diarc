/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import java.util.List;
import java.util.Objects;

/** Factory methods for ASL control EventSpecs. */
public final class AslControlSpecFactory {
  private AslControlSpecFactory() {
  }

  public enum Marker {
    IF("if"),
    THEN("then"),
    ELSEIF("elseif"),
    ELSE("else"),
    ENDIF("endif"),
    WHILE("while"),
    DO("do"),
    ENDWHILE("endwhile"),
    TRUE("true"),
    FALSE("false"),
    EVALUATE("evaluate"),
    OR("or"),
    ENDOR("endor"),
    AND("and"),
    ENDAND("endand"),
    NOT("not"),
    ENDNOT("endnot"),
    DECLARE("declare"),
    FOR("for"),
    FOREACH("foreach"),
    BLOCK("block"),
    ENDBLOCK("endblock"),
    ENDFOR("endfor"),
    TRY("try"),
    CATCH("catch"),
    FINALLY("finally"),
    ENDTRY("endtry"),
    ASYNC("async"),
    ENDASYNC("endasync"),
    NOINTERRUPT("nointerrupt"),
    ENDNOINTERRUPT("endnointerrupt");

    private final String command;

    Marker(String command) {
      this.command = command;
    }

    public String command() {
      return command;
    }

    static Marker fromCommand(String command) {
      for (Marker marker : values()) {
        if (marker.command.equals(command)) {
          return marker;
        }
      }
      return null;
    }
  }

  public static EventSpec control(Marker marker) {
    Objects.requireNonNull(marker, "marker");
    return new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand(marker.command()).build();
  }

  public static EventSpec control(Marker marker, List<String> inputArgs, List<String> returnArgs) {
    Objects.requireNonNull(marker, "marker");
    return new EventSpec.Builder(EventSpec.EventType.CONTROL)
        .setCommand(marker.command())
        .setInputArgs(inputArgs == null ? List.of() : inputArgs)
        .setReturnArgs(returnArgs == null ? List.of() : returnArgs)
        .build();
  }
}
