/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.execution.control.ForContext;
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
    ArrayDeque<Integer> ifGuardStarts = new ArrayDeque<>();
    ArrayDeque<Integer> whileGuardStarts = new ArrayDeque<>();

    for (int i = 0; i < specs.size(); i++) {
      EventSpec spec = specs.get(i);
      if (spec.getType() == EventSpec.EventType.CONTROL) {
        String command = spec.getCommand();
        switch (command) {
          case "if" -> {
            ifGuardStarts.push(i);
            checkRefs(i, spec, stack, spec.getInputArgs());
            checkRefs(i, spec, stack, spec.getReturnArgs());
            continue;
          }
          case "for" -> {
            validateForSignature(i, spec);
            push(stack, "for");
            if (!spec.getInputArgs().isEmpty()) {
              String loopVar = spec.getInputArgs().get(0);
              if (isLocal(loopVar) && !inScope(stack, loopVar)) {
                declare(i, spec, stack.peek(), loopVar);
              }
            }
            checkForReferences(i, spec, stack);
            continue;
          }
          case "foreach" -> {
            validateForeachSignature(i, spec);
            push(stack, "foreach");
            if (!spec.getInputArgs().isEmpty()) {
              String loopVar = spec.getInputArgs().get(0);
              if (isLocal(loopVar) && !inScope(stack, loopVar)) {
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
            if ("then".equals(command)) {
              if (ifGuardStarts.isEmpty()) {
                throw fail(i, spec, "if/elseif branch marker without preceding condition");
              }
              int guardStart = ifGuardStarts.pop();
              if (i <= guardStart + 1) {
                throw fail(i, spec, "missing if/elseif guard expression");
              }
            } else if ("do".equals(command)) {
              if (whileGuardStarts.isEmpty()) {
                throw fail(i, spec, "while body marker without preceding condition");
              }
              int guardStart = whileGuardStarts.pop();
              if (i <= guardStart + 1) {
                throw fail(i, spec, "missing while guard expression");
              }
            }
            push(stack, command);
            continue;
          }
          case "elseif", "else" -> {
            popUntilOneOf(i, stack, Set.of("then"));
            if ("elseif".equals(command)) {
              ifGuardStarts.push(i);
            }
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
            if (!whileGuardStarts.isEmpty()) {
              throw fail(i, spec, "while missing do marker");
            }
            popUntilOneOf(i, stack, Set.of("do"));
            continue;
          }
          case "endif" -> {
            if (!ifGuardStarts.isEmpty()) {
              throw fail(i, spec, "if/elseif missing then marker");
            }
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
            validateDeclaration(i, spec, stack.peek());
            continue;
          }
          case "assign", "evaluate" -> {
            if (!spec.getInputArgs().isEmpty()) {
              checkExpression(i, spec, stack, spec.getInputArgs().get(0));
            }
            checkRefs(i, spec, stack, spec.getReturnArgs());
            continue;
          }
          case "while" -> {
            whileGuardStarts.push(i);
            checkRefs(i, spec, stack, spec.getInputArgs());
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
    AslExpression parsed;
    try {
      parsed = new AslExpression(expression);
    } catch (IllegalArgumentException e) {
      throw fail(index, spec, "malformed expression: " + e.getMessage());
    }
    checkRefs(index, spec, stack, parsed.variables());
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

  private static void validateDeclaration(int index, EventSpec spec, ScopeFrame frame) {
    if (spec.getInputArgs().size() < 2) {
      throw fail(index, spec, "declare requires variable name and java type");
    }
    String name = spec.getInputArgs().get(0);
    String javaType = spec.getInputArgs().get(1);
    if (!isScriptVariable(name)) {
      throw fail(index, spec, "invalid declaration name: " + name);
    }
    if (javaType == null || javaType.isBlank()) {
      throw fail(index, spec, "invalid declaration type for " + name + ": " + javaType);
    }
    if (!isResolvableJavaType(javaType)) {
      throw fail(index, spec, "unresolvable declaration type for " + name + ": " + javaType);
    }
    declare(index, spec, frame, name);
  }

  private static void validateForSignature(int index, EventSpec spec) {
    List<String> args = spec.getInputArgs();
    if (args.size() != 4 && args.size() != 5) {
      throw fail(index, spec, "for requires 4 or 5 input args");
    }
    if (!isScriptVariable(args.get(0))) {
      throw fail(index, spec, "for loop variable must be a script variable");
    }
    String comparator = args.get(args.size() == 5 ? 2 : 1);
    if (comparator == null || comparator.isBlank()) {
      throw fail(index, spec, "for comparator is required");
    }
    String update = args.get(args.size() - 1);
    if (!ForContext.isIncrementOperator(update)) {
      throw fail(index, spec, "invalid for update expression: " + update);
    }
  }

  private static void validateForeachSignature(int index, EventSpec spec) {
    List<String> args = spec.getInputArgs();
    if (args.size() != 2) {
      throw fail(index, spec, "foreach requires exactly 2 input args");
    }
    if (!isScriptVariable(args.get(0)) || !isScriptVariable(args.get(1))) {
      throw fail(index, spec, "foreach requires script variables for element and collection");
    }
  }

  private static void checkForReferences(int index, EventSpec spec, ArrayDeque<ScopeFrame> stack) {
    List<String> args = spec.getInputArgs();
    if (args.size() < 4) {
      return;
    }
    int compareValueIndex = args.size() == 5 ? 3 : 2;
    String compareValue = args.get(compareValueIndex);
    if (isScriptVariable(compareValue)) {
      checkRefs(index, spec, stack, List.of(compareValue));
    } else {
      checkRefsInText(index, spec, stack, compareValue);
    }
    String update = args.get(args.size() - 1);
    checkRefsInText(index, spec, stack, update);
    if (args.size() == 5) {
      String init = args.get(1);
      if (isScriptVariable(init)) {
        checkRefs(index, spec, stack, List.of(init));
      } else {
        checkRefsInText(index, spec, stack, init);
      }
    }
  }

  private static void checkRefsInText(int index, EventSpec spec, ArrayDeque<ScopeFrame> stack, String text) {
    List<String> refs = scriptVariablesInText(text);
    if (!refs.isEmpty()) {
      checkRefs(index, spec, stack, refs);
    }
  }

  private static List<String> scriptVariablesInText(String text) {
    if (text == null || text.isEmpty()) {
      return List.of();
    }
    List<String> vars = new ArrayList<>();
    int i = 0;
    boolean inDouble = false;
    boolean inSingle = false;
    while (i < text.length()) {
      char c = text.charAt(i);
      if (c == '\\') {
        i += 2;
        continue;
      }
      if (!inSingle && c == '"') {
        inDouble = !inDouble;
        i++;
        continue;
      }
      if (!inDouble && c == '\'') {
        inSingle = !inSingle;
        i++;
        continue;
      }
      if (!inDouble && !inSingle && (c == '!' || c == '?')) {
        int start = i;
        i++;
        while (i < text.length()) {
          char next = text.charAt(i);
          if (Character.isLetterOrDigit(next) || next == '_') {
            i++;
          } else {
            break;
          }
        }
        if (i > start + 1) {
          vars.add(text.substring(start, i));
        }
        continue;
      }
      i++;
    }
    return vars;
  }

  private static boolean isResolvableJavaType(String rawType) {
    String type = rawType.trim();
    int genericStart = type.indexOf('<');
    if (genericStart >= 0) {
      type = type.substring(0, genericStart);
    }
    return switch (type) {
      case "byte", "short", "int", "long", "float", "double", "boolean", "char" -> true;
      default -> {
        try {
          Class.forName(type);
          yield true;
        } catch (ClassNotFoundException e) {
          yield false;
        }
      }
    };
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
