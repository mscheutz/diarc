/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import java.math.BigDecimal;

/** Checked assignment shared by expressions and explicit asynchronous results. */
public final class AslValues {
  private AslValues() { }

  public static Class<?> boxed(Class<?> type) {
    if (type == boolean.class) return Boolean.class;
    if (type == byte.class) return Byte.class;
    if (type == short.class) return Short.class;
    if (type == int.class) return Integer.class;
    if (type == long.class) return Long.class;
    if (type == float.class) return Float.class;
    if (type == double.class) return Double.class;
    if (type == char.class) return Character.class;
    return type;
  }

  public static Object convert(Object value, Class<?> type) {
    type = boxed(type);
    if (value == null) {
      throw new IllegalArgumentException("Cannot assign an unbound value to " + type.getTypeName());
    }
    if (type.isInstance(value)) return value;
    if (value instanceof Number && Number.class.isAssignableFrom(type)) {
      BigDecimal number = number(value);
      if (type == Byte.class) return number.byteValueExact();
      if (type == Short.class) return number.shortValueExact();
      if (type == Integer.class) return number.intValueExact();
      if (type == Long.class) return number.longValueExact();
      if (type == Double.class && Double.isFinite(number.doubleValue())) return number.doubleValue();
      if (type == Float.class && Float.isFinite(number.floatValue())) return number.floatValue();
    }
    if (type == Symbol.class && value instanceof String) return Factory.createSymbol((String) value);
    throw new IllegalArgumentException("Cannot assign " + value.getClass().getTypeName() + " to " + type.getTypeName());
  }

  static BigDecimal number(Object value) {
    if (!(value instanceof Number)) throw new IllegalArgumentException("Expected a numeric operand, got " + value);
    return value instanceof BigDecimal ? (BigDecimal) value : new BigDecimal(value.toString());
  }

  static boolean bool(Object value) {
    if (!(value instanceof Boolean)) throw new IllegalArgumentException("Expected a Boolean operand, got " + value);
    return (Boolean) value;
  }

  public static String stringLiteral(String text) {
    StringBuilder value = new StringBuilder();
    for (int i = 1; i < text.length() - 1; i++) {
      char next = text.charAt(i);
      if (next == '\\') next = text.charAt(++i);
      value.append(next);
    }
    return value.toString();
  }
}
