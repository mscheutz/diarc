/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant.util;

import edu.tufts.hrilab.fol.Symbol;
import java.util.regex.Pattern;

public class Utilities {
  /**
   * Check if a single descriptor is a POWER object ref that
   * matches "[kbName]_[number]" (e.g., object_3).
   * @param descriptor
   * @return
   */
  public static boolean isReference(Symbol descriptor, String kbName) {
    Pattern p = Pattern.compile("^" + kbName + "_\\d+$");
    if (p.matcher(descriptor.getName()).matches()) {
      return true;
    }
    return false;
  }

  /**
   * Converts a primitive class to the corresponding Object class otherwise leaves it.
   * @param c
   * @return
   */
  static final public Class primitiveToObject(Class c) {
    if (c.isPrimitive()) {
      if (c == int.class) {
        return java.lang.Integer.class;
      } else if (c == double.class) {
        return java.lang.Double.class;
      } else if (c == boolean.class) {
        return java.lang.Boolean.class;
      } else if (c == byte.class) {
        return java.lang.Byte.class;
      } else if (c == char.class) {
        return java.lang.Character.class;
      } else if (c == long.class) {
        return java.lang.Long.class;
      } else if (c == short.class) {
        return java.lang.Short.class;
      } else if (c == float.class) {
        return java.lang.Float.class;
      } else {
        // must be void type
        return java.lang.Void.class;
      }
    }
    return c;
  }

}
