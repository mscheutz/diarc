/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/** Static lexical-scope checks for ASL EventSpec sequences. */
public final class AslScopeValidator {
  private AslScopeValidator() {
  }

  public static final class ValidationException extends IllegalArgumentException {
    public ValidationException(String message) {
      super(message);
    }
  }

  public static void validate(List<EventSpec> specs) {
    validate(specs, List.of());
  }

  public static void validate(List<EventSpec> specs, List<String> predeclaredLocals) {
    Objects.requireNonNull(specs, "specs");
    AslBlockStructure.validateControlFlow(specs);

    ArrayDeque<ScopeFrame> stack = new ArrayDeque<>();
    ScopeFrame root = new ScopeFrame("root");
    for (String local : predeclaredLocals == null ? List.<String>of() : predeclaredLocals) {
      if (isLocal(local)) {
        root.locals.add(local);
      }
    }
    stack.push(root);

    for (int i = 0; i < specs.size(); i++) {
      EventSpec spec = specs.get(i);
      if (spec.getType() == EventSpec.EventType.CONTROL) {
        String command = spec.getCommand();
        switch (command) {
          case "for" -> {
            push(stack, "for");
            if (!spec.getInputArgs().isEmpty()) {
              String loopVar = spec.getInputArgs().get(0);
              if (isLocal(loopVar)) {
                declare(i, spec, stack.peek(), loopVar);
              }
            }
            checkRefs(i, spec, stack, spec.getInputArgs().subList(Math.min(1, spec.getInputArgs().size()), spec.getInputArgs().size()));
            continue;
          }
          case "foreach" -> {
            push(stack, "foreach");
            if (!spec.getInputArgs().isEmpty()) {
              String loopVar = spec.getInputArgs().get(0);
              if (isLocal(loopVar)) {
                declare(i, spec, stack.peek(), loopVar);
              }
            }
            checkRefs(i, spec, stack, spec.getInputArgs().subList(Math.min(1, spec.getInputArgs().size()), spec.getInputArgs().size()));
            continue;
          }
          case "try", "async", "nointerrupt" -> {
            push(stack, command);
            checkRefs(i, spec, stack, spec.getInputArgs());
            checkRefs(i, spec, stack, spec.getReturnArgs());
            continue;
          }
          case "then", "do", "block" -> {
            push(stack, command);
            continue;
          }
          case "elseif", "else" -> {
            popUntilOneOf(i, stack, Set.of("then"));
            if ("else".equals(command)) {
              push(stack, "else");
            }
            continue;
          }
          case "catch", "finally" -> {
            while (!stack.isEmpty() && !"root".equals(stack.peek().kind)
                && !"try".equals(stack.peek().kind) && !"catch".equals(stack.peek().kind)) {
              stack.pop();
            }
            if (!stack.isEmpty() && "catch".equals(stack.peek().kind)) {
              stack.pop();
            }
            if (stack.isEmpty() || "root".equals(stack.peek().kind)) {
              throw fail(i, spec, "scope stack mismatch");
            }
            push(stack, command);
            if ("catch".equals(command) && !spec.getInputArgs().isEmpty()) {
              List<String> catchArgs = spec.getInputArgs();
              for (int j = 0; j < catchArgs.size(); j++) {
                String arg = catchArgs.get(j);
                if (j == 1 && isLocal(arg)) {
                  declare(i, spec, stack.peek(), arg);
                } else {
                  checkRefs(i, spec, stack, List.of(arg));
                }
              }
            } else {
              checkRefs(i, spec, stack, spec.getInputArgs());
            }
            continue;
          }
          case "endblock" -> {
            popExpected(i, stack, "block");
            continue;
          }
          case "endwhile" -> {
            popUntilOneOf(i, stack, Set.of("do"));
            continue;
          }
          case "endif" -> {
            popUntilOneOf(i, stack, Set.of("then", "else"));
            continue;
          }
          case "endfor" -> {
            if (!stack.isEmpty() && "block".equals(stack.peek().kind)) {
              stack.pop();
            }
            popUntilOneOf(i, stack, Set.of("for", "foreach"));
            continue;
          }
          case "endasync" -> {
            popUntilOneOf(i, stack, Set.of("async"));
            continue;
          }
          case "endnointerrupt" -> {
            popUntilOneOf(i, stack, Set.of("nointerrupt"));
            continue;
          }
          case "endtry" -> {
            while (!stack.isEmpty() && !"root".equals(stack.peek().kind)
                && !"try".equals(stack.peek().kind)
                && !"catch".equals(stack.peek().kind)
                && !"finally".equals(stack.peek().kind)) {
              stack.pop();
            }
            if (!stack.isEmpty() && ("catch".equals(stack.peek().kind) || "finally".equals(stack.peek().kind))) {
              stack.pop();
            }
            popExpected(i, stack, "try");
            continue;
          }
          case "declare" -> {
            if (spec.getInputArgs().isEmpty()) {
              throw fail(i, spec, "declare requires variable name");
            }
            declare(i, spec, stack.peek(), spec.getInputArgs().get(0));
            continue;
          }
          case "assign", "evaluate" -> {
            if (!spec.getInputArgs().isEmpty()) {
              checkExpression(i, spec, stack, spec.getInputArgs().get(0));
            }
            checkRefs(i, spec, stack, spec.getReturnArgs());
            continue;
          }
          default -> {
            checkRefs(i, spec, stack, spec.getInputArgs());
            checkRefs(i, spec, stack, spec.getReturnArgs());
            continue;
          }
        }
      }
      checkRefs(i, spec, stack, spec.getInputArgs());
      checkRefs(i, spec, stack, spec.getReturnArgs());
    }
  }

  private static void checkExpression(int index, EventSpec spec, ArrayDeque<ScopeFrame> stack, String expression) {
    try {
      AslExpression parsed = new AslExpression(expression);
      checkRefs(index, spec, stack, parsed.variables());
    } catch (RuntimeException | Error ignored) {
      // expression parser will validate at runtime/parser stage; scope checks only if parseable here
    }
  }

  private static void checkRefs(int index, EventSpec spec, ArrayDeque<ScopeFrame> stack, List<String> refs) {
    for (String token : refs) {
      if (!isScriptVariable(token)) {
        continue;
      }
      if (isGlobal(token)) {
        continue;
      }
      if (!inScope(stack, token)) {
        throw fail(index, spec, "variable used outside lexical scope or before declaration: " + token);
      }
    }
  }

  private static void declare(int index, EventSpec spec, ScopeFrame frame, String name) {
    if (!isScriptVariable(name)) {
      throw fail(index, spec, "invalid declaration name: " + name);
    }
    if (!frame.locals.add(name)) {
      throw fail(index, spec, "duplicate declaration in same lexical scope: " + name);
    }
  }

  private static boolean inScope(ArrayDeque<ScopeFrame> stack, String name) {
    for (ScopeFrame frame : stack) {
      if (frame.locals.contains(name)) {
        return true;
      }
    }
    return false;
  }

  private static void push(ArrayDeque<ScopeFrame> stack, String kind) {
    stack.push(new ScopeFrame(kind));
  }

  private static void popExpected(int index, ArrayDeque<ScopeFrame> stack, String kind) {
    if (stack.isEmpty() || !kind.equals(stack.peek().kind)) {
      throw fail(index, null, "scope stack mismatch; expected " + kind);
    }
    stack.pop();
  }

  private static void popUntilOneOf(int index, ArrayDeque<ScopeFrame> stack, Set<String> allowedTopKinds) {
    while (!stack.isEmpty() && !allowedTopKinds.contains(stack.peek().kind)) {
      if ("root".equals(stack.peek().kind)) {
        break;
      }
      stack.pop();
    }
    if (stack.isEmpty() || "root".equals(stack.peek().kind)) {
      throw fail(index, null, "scope stack mismatch");
    }
    stack.pop();
  }

  private static boolean isScriptVariable(String value) {
    return value != null && value.length() > 1 && (value.charAt(0) == '!' || value.charAt(0) == '?');
  }

  private static boolean isLocal(String value) {
    return value != null && value.startsWith("!");
  }

  private static boolean isGlobal(String value) {
    return value != null && value.startsWith("?");
  }

  private static ValidationException fail(int index, EventSpec spec, String detail) {
    StringBuilder message = new StringBuilder("Scope validation failed at EventSpec index ").append(index);
    if (spec != null) {
      message.append(" [");
      message.append(spec.getType()).append(":").append(spec.getCommand());
      SourceLocation location = spec.getSourceLocation();
      if (location != null) {
        message.append(" @").append(location.source()).append(":").append(location.line()).append(":").append(location.column());
      }
      message.append("]");
    }
    message.append(": ").append(detail);
    return new ValidationException(message.toString());
  }

  private static final class ScopeFrame {
    private final String kind;
    private final Set<String> locals = new HashSet<>();

    private ScopeFrame(String kind) {
      this.kind = kind;
    }
  }
}
