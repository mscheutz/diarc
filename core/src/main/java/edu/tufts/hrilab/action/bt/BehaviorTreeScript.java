/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Set;

/**
 * Compiles the BehaviorTree.CPP 4.10 expression language to native ASL steps.
 * Evaluation order, conditional expressions and comparison-chain short circuiting
 * are represented by ASL steps; runtime calls perform individual scalar operations.
 *
 * <p>Supported syntax includes decimal/hex integers, decimal reals with optional
 * exponents, numeric booleans, raw single-quoted strings, blackboard identifiers
 * (ASCII letters/underscore followed by letters/underscore/digits), parentheses,
 * unary {@code - ! ~}, arithmetic
 * {@code + - * /}, concatenation {@code ..}, bitwise {@code & | ^}, comparisons
 * (including chains), {@code && ||}, ternaries, and assignments
 * {@code := = += -= *= /=}. Semicolons between expressions are optional, but
 * leading semicolons and empty scripts are invalid. Strings have no escape syntax.
 * Comments, calls, indexing, member access, unary plus, modulo and shifts are not
 * language constructs. A leading {@code @} resolves an identifier in the root
 * blackboard, bypassing subtree remapping.
 *
 * <p>The execution receiver must provide {@code decoded(String)},
 * {@code scriptNumber(String)}, {@code scriptRead(String)},
 * {@code scriptPrepareAssignment(String, String)},
 * {@code scriptAssign(String, String, Object)},
 * {@code scriptBinary(String, Object, Object)},
 * {@code scriptUnary(String, Object)}, {@code scriptIdentity(Object)},
 * {@code scriptTruthy(Object)} and {@code scriptConditionalTruthy(Object)}.
 * The last two return Boolean; the other scalar
 * methods return Object (decoded returns String, preparation returns void).
 * Identity preserves its argument.
 * Number parses an unsigned decimal/hex integer as Long or a real as Double.
 * Literal booleans are emitted as double 1/0. Operators and keys are decoded
 * separately, never passed as source text to an evaluator.
 *
 * <p>Runtime contracts follow upstream {@code scripting/operators.hpp}:
 * <ul>
 * <li>{@code ExprName}: enum lookup precedes blackboard lookup; missing names
 * throw. Existing uninitialized entries remain distinguishable from missing names:
 * reads may return an uninitialized sentinel, which scalar operations reject
 * after evaluating their operands. A read snapshots the scalar, so subsequent
 * RHS writes do not alter it.</li>
 * <li>{@code ExprBinaryArithmetic}: both operands are evaluated left-to-right,
 * including for logical operations, before checking for uninitialized values.
 * Arithmetic promotes numbers to double, including IEEE division without a
 * zero-divisor check. Plus additionally concatenates two strings; dot-dot accepts
 * two strings or a string/number pair, with upstream numeric-to-string conversion,
 * but not two numbers. Other string arithmetic is invalid.</li>
 * <li>Binary bitwise operations use checked signed-64-bit integer conversions:
 * fractional or out-of-range operands throw. Numeric logical operands use checked
 * {@code Any.cast<bool>()}, not general string/nonzero truth conversion. Logical
 * and bitwise results are doubles, with logical results exactly 0.0 or 1.0.</li>
 * <li>{@code ExprUnaryArithmetic}: operands must be numeric; minus returns a
 * double, logical-not uses numeric zero/nonzero, and complement range-checks the double,
 * truncates toward zero to signed int64, complements, then returns a double.
 * All unary operators reject strings.</li>
 * <li>{@code ExprComparison}/{@code IsSame}: numbers become doubles. Equality is
 * {@code abs(left-right) <= 0x1.0p-23}; inequality negates that test, with no
 * relative scaling or infinity special case. Strings compare exactly/lexically.
 * Mixed string/number comparison converts the string through
 * {@code StringToDouble}: exact "true"/"false", then registered enums, then the
 * upstream numeric cast (conversion failures throw). Ordering uses the negated
 * opposite relation: less is {@code !(left >= right)}, greater is
 * {@code !(left <= right)}, less-equal is {@code !(left > right)}, and greater-equal
 * is {@code !(left < right)}. This matters for NaN. Results are double 1/0.
 * Comparison chains evaluate the first pair and stop on failure; each later
 * operand is evaluated only after all previous comparisons succeeded.</li>
 * <li>{@code ExprIf}: a nonempty string selects the true branch without numeric
 * conversion; otherwise the condition is converted to double and tested against
 * zero. An empty string thus still requires the upstream numeric conversion
 * (it is not unconditionally false). Only the selected branch is evaluated.
 * {@code scriptConditionalTruthy} must implement that condition conversion.
 * In contrast, {@code scriptTruthy} implements the checked numeric-to-bool cast
 * used by predicate hooks; it rejects strings instead of treating them as true.</li>
 * <li>{@code ExprAssignment}: preparation resolves the target before the RHS,
 * creating an uninitialized entry only for colon-equal; all other operations
 * reject missing targets before any RHS side effects. Assignment then rejects an
 * uninitialized RHS. An unconstrained empty target acquires the RHS type.
 * Existing targets use upstream checked copy/type conversion; strings assigned
 * to nonstrings try the registered port converter, then StringToDouble for
 * numeric targets, otherwise fail. Compound operations require initialized
 * targets; numeric RHS operations promote to double and copy back with checked
 * conversion. String RHS supports only plus-equal, concatenating the target's
 * upstream string cast. Writes update entry metadata and return the stored
 * value. Preparation must not overwrite an existing target.</li>
 * </ul>
 */
public final class BehaviorTreeScript {
  private static final Set<String> ASSIGNMENTS = Set.of(":=", "=", "+=", "-=", "*=", "/=");
  private static final Set<String> COMPARISONS = Set.of("==", "!=", "<", ">", "<=", ">=");
  private static final int MAX_TOKENS = 2048;
  private static final int MAX_DEPTH = 128;

  private BehaviorTreeScript() {
  }

  /**
   * Returns declarations followed by instructions assigning the last expression
   * to an already declared result variable. A predicate result is a Boolean;
   * otherwise it is an Object. Prefixes must start with a lowercase ASCII letter
   * (an optional leading {@code !} is accepted). Use a distinct prefix per call,
   * and hoist the returned declarations to the enclosing ASL action's body.
   *
   * @throws IllegalArgumentException for invalid syntax or exceeded compiler bounds
   */
  public static String compile(String source, String variablePrefix, String resultVariable,
      boolean predicate) {
    if (source == null || source.length() > 65536) {
      throw new IllegalArgumentException("Script is null or exceeds 65536 characters");
    }
    if (variablePrefix != null && variablePrefix.startsWith("!")) {
      variablePrefix = variablePrefix.substring(1);
    }
    if (variablePrefix == null || !variablePrefix.matches("[a-z][A-Za-z_0-9]*")
        || resultVariable == null || !resultVariable.matches("[!?][a-z][A-Za-z_0-9]*")) {
      throw new IllegalArgumentException("Invalid ASL script prefix or result variable");
    }
    Parser parser = new Parser(source);
    Generator generator = new Generator(variablePrefix, resultVariable);
    String result = null;
    while (!parser.atEnd()) {
      result = generator.emit(parser.expression(0));
      while (parser.take(";")) {
        // Semicolons are optional between expressions, and may be repeated.
      }
    }
    if (result == null) {
      throw new IllegalArgumentException("Empty script");
    }
    generator.line(resultVariable + " = "
        + generator.invoke(predicate ? "scriptTruthy" : "scriptIdentity", result) + ";");
    return generator.declarations.toString() + generator.body;
  }

  private record Token(String text, int position, Kind kind) {
  }

  private enum Kind { NUMBER, STRING, NAME, OPERATOR, END }

  private record Expression(String operator, String text, List<Expression> operands) {
    static Expression atom(String operator, String text) {
      return new Expression(operator, text, List.of());
    }

    static Expression operation(String operator, Expression... operands) {
      return new Expression(operator, null, List.of(operands));
    }
  }

  private static final class Parser {
    private final List<Token> tokens = new ArrayList<>();
    private int index;
    private int depth;

    Parser(String source) {
      tokenize(source);
    }

    private IllegalArgumentException error(String message) {
      return new IllegalArgumentException("Script at position " + tokens.get(index).position
          + ": " + message);
    }

    boolean atEnd() {
      return tokens.get(index).kind == Kind.END;
    }

    boolean take(String text) {
      if (tokens.get(index).kind == Kind.OPERATOR && tokens.get(index).text.equals(text)) {
        index++;
        return true;
      }
      return false;
    }

    private void expect(String text) {
      if (!take(text)) {
        throw error("Expected '" + text + "'");
      }
    }

    Expression expression(int minimum) {
      if (++depth > MAX_DEPTH) {
        throw error("Expression nesting exceeds " + MAX_DEPTH);
      }
      try {
        Expression left = operand();
        while (tokens.get(index).kind == Kind.OPERATOR
            && precedence(tokens.get(index).text) >= minimum) {
          String operator = tokens.get(index++).text;
          int level = precedence(operator);
          if (ASSIGNMENTS.contains(operator)) {
            if (!left.operator.equals("read")) {
              throw error("Assignment target must be a blackboard identifier");
            }
            return new Expression("assign", operator,
                List.of(left, expression(0)));
          }
          if (operator.equals("?")) {
            Expression yes = expression(0);
            expect(":");
            return Expression.operation("?:", left, yes, expression(level));
          }
          if (COMPARISONS.contains(operator)) {
            List<Expression> values = new ArrayList<>();
            List<String> operators = new ArrayList<>();
            values.add(left);
            operators.add(operator);
            values.add(expression(level + 1));
            while (tokens.get(index).kind == Kind.OPERATOR
                && COMPARISONS.contains(tokens.get(index).text)) {
              operators.add(tokens.get(index++).text);
              values.add(expression(level + 1));
            }
            left = new Expression("compare", String.join(" ", operators), List.copyOf(values));
          } else {
            left = Expression.operation(operator, left, expression(level + 1));
          }
        }
        return left;
      } finally {
        depth--;
      }
    }

    private Expression operand() {
      Token token = tokens.get(index);
      if (take("(")) {
        Expression result = expression(0);
        expect(")");
        return result;
      }
      if (Set.of("-", "!", "~").contains(token.text) && token.kind == Kind.OPERATOR) {
        index++;
        return new Expression("unary", token.text, List.of(expression(20)));
      }
      if (token.kind == Kind.NUMBER || token.kind == Kind.STRING || token.kind == Kind.NAME) {
        index++;
        if (token.kind == Kind.STRING) {
          return Expression.atom("string", token.text);
        }
        if (token.kind == Kind.NUMBER) {
          return Expression.atom("number", token.text);
        }
        if (token.text.equals("true") || token.text.equals("false")) {
          return Expression.atom("number", token.text.equals("true") ? "1.0" : "0.0");
        }
        return Expression.atom("read", token.text);
      }
      throw error("Expected an operand, got '" + token.text + "'");
    }

    private static int precedence(String operator) {
      if (ASSIGNMENTS.contains(operator)) {
        return 2;
      }
      if (COMPARISONS.contains(operator)) {
        return 10;
      }
      return switch (operator) {
        case "?" -> 4;
        case "||" -> 6;
        case "&&" -> 8;
        case "|", "^" -> 12;
        case "&" -> 14;
        case "+", "-", ".." -> 16;
        case "*", "/" -> 18;
        default -> -1;
      };
    }

    private void tokenize(String source) {
      int cursor = 0;
      while (cursor < source.length()) {
        char ch = source.charAt(cursor);
        if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
          cursor++;
          continue;
        }
        int start = cursor++;
        Kind kind;
        String text;
        if (ch == '\'') {
          int end = source.indexOf('\'', cursor);
          if (end < 0) {
            throw new IllegalArgumentException("Unterminated script string at position " + start);
          }
          text = source.substring(cursor, end);
          cursor = end + 1;
          kind = Kind.STRING;
        } else if (digit(ch)) {
          boolean hex = ch == '0' && cursor < source.length()
              && (source.charAt(cursor) == 'x' || source.charAt(cursor) == 'X');
          if (hex) {
            cursor++;
            while (cursor < source.length() && hexDigit(source.charAt(cursor))) {
              cursor++;
            }
          } else {
            while (cursor < source.length() && digit(source.charAt(cursor))) {
              cursor++;
            }
            if (cursor < source.length() && source.charAt(cursor) == '.'
                && !(cursor + 1 < source.length() && source.charAt(cursor + 1) == '.')) {
              cursor++;
              int fraction = cursor;
              while (cursor < source.length() && digit(source.charAt(cursor))) {
                cursor++;
              }
              if (fraction == cursor) {
                throw new IllegalArgumentException("Incomplete real at position " + start);
              }
            }
            if (cursor < source.length()
                && (source.charAt(cursor) == 'e' || source.charAt(cursor) == 'E')) {
              cursor++;
              if (cursor < source.length()
                  && (source.charAt(cursor) == '+' || source.charAt(cursor) == '-')) {
                cursor++;
              }
              int exponent = cursor;
              while (cursor < source.length() && digit(source.charAt(cursor))) {
                cursor++;
              }
              if (exponent == cursor) {
                throw new IllegalArgumentException("Incomplete exponent at position " + start);
              }
            }
          }
          if (cursor < source.length() && (identifierStart(source.charAt(cursor))
              || (hex && source.charAt(cursor) == '.'))) {
            throw new IllegalArgumentException("Invalid number at position " + start);
          }
          text = source.substring(start, cursor);
          try {
            if (hex) {
              Long.parseLong(text.substring(2), 16);
            } else if (!text.contains(".") && !text.contains("e") && !text.contains("E")) {
              Long.parseLong(text);
            } else if (!Double.isFinite(Double.parseDouble(text))) {
              throw new NumberFormatException();
            }
          } catch (NumberFormatException invalid) {
            throw new IllegalArgumentException("Number out of range at position " + start, invalid);
          }
          kind = Kind.NUMBER;
        } else if (identifierStart(ch) || ch == '@') {
          if (ch == '@' && (cursor >= source.length() || !identifierStart(source.charAt(cursor)))) {
            throw new IllegalArgumentException("Invalid global identifier at position " + start);
          }
          while (cursor < source.length() && identifierPart(source.charAt(cursor))) {
            cursor++;
          }
          text = source.substring(start, cursor);
          kind = Kind.NAME;
        } else {
          text = source.substring(start, cursor);
          if (cursor < source.length()) {
            String pair = source.substring(start, cursor + 1);
            if (ASSIGNMENTS.contains(pair) || COMPARISONS.contains(pair)
                || Set.of("&&", "||", "..").contains(pair)) {
              text = pair;
              cursor++;
            }
          }
          if (text.length() == 1 && "+-*/&|^~!<>=?:();".indexOf(ch) < 0) {
            throw new IllegalArgumentException("Unsupported token at position " + start + ": " + ch);
          }
          kind = Kind.OPERATOR;
        }
        if (tokens.size() >= MAX_TOKENS) {
          throw new IllegalArgumentException("Script exceeds " + MAX_TOKENS + " tokens");
        }
        tokens.add(new Token(text, start, kind));
      }
      tokens.add(new Token("", cursor, Kind.END));
    }

    private static boolean digit(char value) {
      return value >= '0' && value <= '9';
    }

    private static boolean hexDigit(char value) {
      return digit(value) || value >= 'a' && value <= 'f' || value >= 'A' && value <= 'F';
    }

    private static boolean identifierStart(char value) {
      return value >= 'a' && value <= 'z' || value >= 'A' && value <= 'Z'
          || value == '_';
    }

    private static boolean identifierPart(char value) {
      return identifierStart(value) || digit(value);
    }
  }

  private static final class Generator {
    private final String prefix;
    private final String resultVariable;
    private final StringBuilder declarations = new StringBuilder();
    private final StringBuilder body = new StringBuilder();
    private int counter;
    private int depth;

    Generator(String prefix, String resultVariable) {
      this.prefix = prefix;
      this.resultVariable = resultVariable;
    }

    private String temporary() {
      String name;
      do {
        name = "!" + prefix + "_" + counter++;
      } while (name.equals(resultVariable));
      declarations.append("java.lang.Object ").append(name).append(";\n");
      return name;
    }

    void line(String instruction) {
      body.append(instruction).append('\n');
    }

    String invoke(String method, String... arguments) {
      return "op:invokeMethod(?execution, \"" + method + "\""
          + (arguments.length == 0 ? "" : ", " + String.join(", ", arguments)) + ")";
    }

    private String store(String value) {
      String variable = temporary();
      line(variable + " = " + value + ";");
      return variable;
    }

    private String text(String value) {
      String encoded = Base64.getEncoder().encodeToString(value.getBytes(StandardCharsets.UTF_8));
      return store(invoke("decoded", "\"" + encoded + "\""));
    }

    private String binary(String operator, String left, String right) {
      return invoke("scriptBinary", text(operator), left, right);
    }

    String emit(Expression expression) {
      if (++depth > MAX_DEPTH) {
        throw new IllegalArgumentException("Expression compilation exceeds nesting bound " + MAX_DEPTH);
      }
      try {
        List<Expression> operands = expression.operands;
        return switch (expression.operator) {
          case "string" -> text(expression.text);
          case "number" -> store(invoke("scriptNumber", "\"" + expression.text + "\""));
          case "read" -> store(invoke("scriptRead", text(expression.text)));
          case "assign" -> {
            String key = text(operands.get(0).text);
            String operator = text(expression.text);
            line(invoke("scriptPrepareAssignment", key, operator) + ";");
            String value = emit(operands.get(1));
            yield store(invoke("scriptAssign", key, operator, value));
          }
          case "unary" -> {
            String value = emit(operands.get(0));
            yield store(invoke("scriptUnary", text(expression.text), value));
          }
          case "?:" -> {
            String condition = emit(operands.get(0));
            String result = temporary();
            line("if (" + invoke("scriptConditionalTruthy", condition) + ") {");
            line(result + " = " + invoke("scriptIdentity", emit(operands.get(1))) + ";");
            line("} else {");
            line(result + " = " + invoke("scriptIdentity", emit(operands.get(2))) + ";");
            line("}");
            yield result;
          }
          case "compare" -> {
            String left = emit(operands.get(0));
            String result = temporary();
            String[] operators = expression.text.split(" ");
            for (int i = 0; i < operators.length; i++) {
              if (i > 0) {
                line("if (" + invoke("scriptTruthy", result) + ") {");
              }
              String right = emit(operands.get(i + 1));
              line(result + " = " + binary(operators[i], left, right) + ";");
              left = right;
            }
            for (int i = 1; i < operators.length; i++) {
              line("}");
            }
            yield result;
          }
          default -> {
            String left = emit(operands.get(0));
            String right = emit(operands.get(1));
            yield store(binary(expression.operator, left, right));
          }
        };
      } finally {
        depth--;
      }
    }
  }
}
