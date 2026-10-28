/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.fol.Factory;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.ParseTree;
import java.math.BigDecimal;
import java.math.MathContext;
import java.util.Objects;

/** Pure, typed expressions. Logical operands remain eager, like existing ASL controls. */
public final class AslExpression {
  private final ASLParser.ValueExpressionContext expression;
  private final java.util.List<String> variables;

  public AslExpression(String source) {
    if (source.length() > 16384) throw new IllegalArgumentException("ASL expression is too long");
    BaseErrorListener errors = new BaseErrorListener() {
      @Override
      public void syntaxError(Recognizer<?, ?> recognizer, Object symbol, int line, int column,
                              String message, RecognitionException exception) {
        throw new IllegalArgumentException("Invalid expression at " + line + ":" + (column + 1) + ": " + message);
      }
    };
    ASLLexer lexer = new ASLLexer(CharStreams.fromString(source));
    lexer.removeErrorListeners();
    lexer.addErrorListener(errors);
    CommonTokenStream tokens = new CommonTokenStream(lexer);
    tokens.fill();
    variables = tokens.getTokens().stream()
        .filter(token -> token.getType() == ASLLexer.LOCALVAR || token.getType() == ASLLexer.GLOBALVAR)
        .map(Token::getText).distinct().collect(java.util.stream.Collectors.toList());
    if (tokens.size() > 2048) throw new IllegalArgumentException("Too many expression tokens");
    int depth = 0;
    int unaryRun = 0;
    for (Token token : tokens.getTokens()) {
      if ("(".equals(token.getText()) && ++depth > 32) {
        throw new IllegalArgumentException("ASL expression nesting exceeds 32");
      }
      if (")".equals(token.getText())) depth--;
      if (java.util.Set.of("~", "+", "-").contains(token.getText())) {
        if (++unaryRun > 64) throw new IllegalArgumentException("Too many unary expression operators");
      } else {
        unaryRun = 0;
      }
    }
    ASLParser parser = new ASLParser(tokens);
    parser.removeErrorListeners();
    parser.addErrorListener(errors);
    expression = parser.expressionUnit().valueExpression();
  }

  public Object evaluate(Context context) {
    return evaluate(expression, context, 0);
  }

  public java.util.List<String> variables() { return variables; }

  private Object evaluate(ParseTree tree, Context context, int depth) {
    if (depth > 512) throw new IllegalArgumentException("ASL expression evaluation is too deeply nested");
    if (tree instanceof ASLParser.ValueAtomContext) {
      ASLParser.ValueAtomContext atom = (ASLParser.ValueAtomContext) tree;
      if (atom.valueExpression() != null) return evaluate(atom.valueExpression(), context, depth + 1);
      String text = atom.getText();
      if (atom.LOCALVAR() != null || atom.GLOBALVAR() != null) {
        Object value = context.readValue(text);
        if (value == null) throw new IllegalArgumentException("Unbound expression variable: " + text);
        return value;
      }
      if (atom.TRUE() != null || atom.FALSE() != null) return Boolean.valueOf(text);
      if (atom.SENTENCE() != null) {
        return AslValues.stringLiteral(text);
      }
      if (atom.NUMBER() != null) {
        if (text.matches(".*[a-zA-Z]$")) {
          if (!text.matches(".*[lLfFdD]$")) throw new IllegalArgumentException("Invalid numeric suffix: " + text);
          text = text.substring(0, text.length() - 1);
        }
        return new BigDecimal(text);
      }
      return Factory.createSymbol(text);
    }
    if (tree instanceof ASLParser.ValueUnaryContext && tree.getChildCount() == 2) {
      Object value = evaluate(tree.getChild(1), context, depth + 1);
      return switch (tree.getChild(0).getText()) {
        case "~" -> !AslValues.bool(value);
        case "+" -> AslValues.number(value);
        case "-" -> AslValues.number(value).negate();
        default -> throw new IllegalArgumentException("Invalid unary expression");
      };
    }
    Object result = evaluate(tree.getChild(0), context, depth + 1);
    for (int i = 1; i < tree.getChildCount(); i += 2) {
      Object right = evaluate(tree.getChild(i + 1), context, depth + 1);
      result = binary(tree.getChild(i).getText(), result, right);
    }
    return result;
  }

  private static Object binary(String operator, Object left, Object right) {
    return switch (operator) {
      case "&&" -> AslValues.bool(left) & AslValues.bool(right);
      case "||" -> AslValues.bool(left) | AslValues.bool(right);
      case "==" -> equal(left, right);
      case "!=" -> !equal(left, right);
      case "gt" -> AslValues.number(left).compareTo(AslValues.number(right)) > 0;
      case "ge", "geq" -> AslValues.number(left).compareTo(AslValues.number(right)) >= 0;
      case "lt" -> AslValues.number(left).compareTo(AslValues.number(right)) < 0;
      case "le", "leq" -> AslValues.number(left).compareTo(AslValues.number(right)) <= 0;
      case "+" -> left instanceof String || right instanceof String
          ? left.toString() + right : AslValues.number(left).add(AslValues.number(right));
      case "-" -> AslValues.number(left).subtract(AslValues.number(right));
      case "*" -> AslValues.number(left).multiply(AslValues.number(right));
      case "/" -> AslValues.number(left).divide(AslValues.number(right), MathContext.DECIMAL128);
      case "%" -> AslValues.number(left).remainder(AslValues.number(right));
      default -> throw new IllegalArgumentException("Unsupported expression operator: " + operator);
    };
  }

  private static boolean equal(Object left, Object right) {
    return left instanceof Number && right instanceof Number
        ? AslValues.number(left).compareTo(AslValues.number(right)) == 0 : Objects.equals(left, right);
  }
}
