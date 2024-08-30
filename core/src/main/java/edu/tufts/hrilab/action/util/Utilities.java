/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.util;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;

import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Evan Krause evan.krause@tufts.edu
 */
public class Utilities {

  protected final static Logger log = LoggerFactory.getLogger(Utilities.class);

  /**
   * Returns the Class object associated with the class or interface with the given string name.
   * This is the same as Class.forName(), but with added support for Java primitives.
   *
   * @param type
   * @return
   */
  public static Class<?> getClass(String type) {
    if (type == null) {
      return null;
    }

    // Support for primitive types
    switch (type) {
      case "int":
        return int.class;
      case "long":
        return long.class;
      case "double":
        return double.class;
      case "float":
        return float.class;
      case "short":
        return short.class;
      case "char":
        return char.class;
      case "byte":
        return byte.class;
      case "boolean":
      case "bool":
        return boolean.class;
      default:
        try {
          return Class.forName(type);
        } catch (ClassNotFoundException e) {
          log.trace("Class not found for type: " + type + ". Returning null.", e);
        }
    }

    return null;
  }

  /**
   * Check is Class is a numeric type (primitive or non-primitive).
   *
   * @param clazz
   * @return
   */
  public static boolean isNumericType(Class<?> clazz) {
    return (clazz.getName().equals("int") || clazz.isAssignableFrom(Integer.class) ||
            clazz.getName().equals("long") || clazz.isAssignableFrom(Long.class) ||
            clazz.getName().equals("short") || clazz.isAssignableFrom(Short.class) ||
            clazz.getName().equals("double") || clazz.isAssignableFrom(Double.class) ||
            clazz.getName().equals("float") || clazz.isAssignableFrom(Float.class));
  }

  /**
   * Check is Class is a boolean type (primitive or non-primitive).
   *
   * @param clazz
   * @return
   */
  public static boolean isBooleanType(Class<?> clazz) {
    return (clazz.getName().equals("boolean") || clazz.isAssignableFrom(Boolean.class));
  }

  private static Class<?> getWrapperClass(Class<?> clazz) {
    switch (clazz.getName()) {
      case "int":
        return Integer.class;
      case "long":
        return Long.class;
      case "double":
        return Double.class;
      case "float":
        return Float.class;
      case "short":
        return Short.class;
      case "char":
        return Character.class;
      case "byte":
        return Byte.class;
      case "bool":
      case "boolean":
        return Boolean.class;
      default:
        log.error("Could not find wrapper class: " + clazz.getName());
        return null;
    }
  }

  /**
   * Checks if value's toString is an integer, long, float, or double.
   *
   * @param value
   * @return
   */
  public static boolean isNumeric(Object value) {
    return value.toString().matches("[-+]?\\d*\\.?\\d+[fFlLdD]?");
  }

  /**
   * Checks if value's toString is a valid boolean value (i.e., "true" or "false").
   *
   * @param value
   * @return
   */
  public static boolean isBoolean(Object value) {
    return (value.toString().equalsIgnoreCase("true") || value.toString().equalsIgnoreCase("false")) ? true : false;
  }

  /**
   * Checks if value's toString is an integer with no decimal place.
   *
   * @param value
   * @return
   */
  public static boolean isInteger(Object value) {
    return value.toString().matches("[-+]?\\d+");
  }

  /**
   * Checks if value's toString is a long that ends "l" or "L" (e.g., 3l, or 3L).
   *
   * @param value
   * @return
   */
  public static boolean isLong(Object value) {
    return value.toString().matches("[-+]?\\d+[lL]");
  }

  /**
   * Checks if value's toString is a float that ends "f" or "F" (e.g., 0.3f, 0.3F).
   *
   * @param value
   * @return
   */
  public static boolean isFloat(Object value) {
    return value.toString().matches("[-+]?\\d*\\.?\\d+[fF]");
  }

  /**
   * Checks if a string is an action script variable
   *
   * @param value variable name
   * @return true if string is variable (starts with ? or !)
   */
  public static boolean isScriptVariable(String value) {
    return value.startsWith("?") || value.startsWith("!");
  }

  /**
   * Checks if a Symbol is an action script variable (of type Variable or starts with ! or ?)
   * TODO: all Symbols that start with ! or ? and aren't of type Variable are being constructed
   * incorrectly. those instances should be fixed and this method removed.
   *
   * @param value symbol to check
   * @return true if string is variable (starts with ? or !)
   */
  public static boolean isScriptVariable(Symbol value) {
    if (value.isVariable() && (value.getName().startsWith("?") || value.getName().startsWith("!"))) {
      return true;
    } else if (value.getName().startsWith("?") || value.getName().startsWith("!")) {
      log.error("[isVariable] Symbols that start with ! or ? should be of type Variable!");
      return true;
    }

    return false;
  }

  /**
   * Checks if a Symbol is an action script local variable
   *
   * @param value Symbol
   * @return true if string is local variable (starts with !)
   */
  public static boolean isLocalVariable(Symbol value) {
    return value.isVariable() && value.getName().startsWith("!");
  }

  /**
   * Checks if a string is an action script local variable
   *
   * @param value variable name
   * @return true if string is local variable (starts with !)
   */
  public static boolean isLocalVariable(String value) {
    return value.startsWith("!");
  }

  /**
   * @return true if class if of integer type
   */
  public static boolean isIntegerType(Class<?> cls) {
    return cls.equals(Integer.class) || cls.equals(int.class);
  }

  /**
   * Convert string to enum type
   *
   * @param enumeration enum
   * @param str         string to convert to enum type
   * @return
   */
  public static <T extends Enum<?>> T strToEnum(Class<T> enumeration, String str) {
    if (str != null) {
      for (T each : enumeration.getEnumConstants()) {
        if (each.name().compareToIgnoreCase(str) == 0) {
          return each;
        }
      }
    }
    log.trace("The specified enum type (" + enumeration.toString() + ") has no constant "
            + "with the specified name (" + str + ") or the specified class object does not "
            + "represent an enum type.");
    return null;
  }

  /**
   * Attempt to convert value to specified type.
   *
   * @param clazz a Java class (for the time being at least)
   * @param value
   * @return
   */
  public static Object convertToType(Class<?> clazz, Object value) {
    Object output;

    if (clazz == null) {
      log.error("Class is null. Returning null.");
      return null;
    }

    log.debug("convertToType type: " + clazz.getName() + " value: " + value);

    // first handle numeric and boolean cases, as those shouldn't have quotes around them.
    // If they do have quotes, they will be treated like a String
    if (clazz == Object.class && (value instanceof String) && isNumeric(value)) {
      // Object should interpreted as Integer/Long/Float/Double, otherwise any number read from text/script will be in string format.
      if (isInteger(value)) {
        output = Integer.valueOf(value.toString());
      } else if (isLong(value)) {
        // first remove the "l" or "L" from the value, as Long.valueOf can't handle it
        String longValue = value.toString();
        longValue = value.toString().substring(0, longValue.length() - 1);
        output = Long.valueOf(longValue);
      } else if (isFloat(value)) {
        output = Float.valueOf(value.toString());
      } else {
        output = Double.valueOf(value.toString());
      }
    } else if (clazz == Object.class && (value instanceof String) && isBoolean(value)) {
      output = Boolean.valueOf(value.toString());
    } else {
      // now handle, non-numeric and non-boolean cases

      // remove enclosing quotes around strings
      if (value != null && (value instanceof String)){
        if( ((String) value).startsWith("\"") && ((String) value).endsWith("\"")) {
          value = ((String) value).substring(1, ((String) value).length() - 1);
        }
        //replace any escaped quotes with regular quotes?
        //TODO:brad: allow for escaping of other characters?
        value = ((String)value).replaceAll("\\\\"+"\\\"","\"");
      }


      if (clazz.isInstance(value)) {
        // if already of target type, we're done
        output = value;
      } else if (clazz.equals(String.class) && value instanceof Symbol) {
        output = ((Symbol) value).toUntypedString();
      } else if (value == null || (value instanceof String && value.toString().equalsIgnoreCase("null"))) {
        // if value is "null" just return null, no need to instantiate null object of specified type
        return null;
      } else if (value instanceof Symbol && ((Symbol) value).getName().startsWith("\"") && ((Symbol) value).getName().endsWith("\"")) {
        //if the Symbol has escape quotes around special characters , take of the quotes and just pass on the string.
        //TODO:brad: better range checking here
        return convertToType(clazz, ((Symbol) value).getName().substring(1, ((Symbol) value).getName().length() - 1));
      } else {
        // value isn't null and isn't already of the correct type, so try to construct object of specified type
        if (clazz.isEnum()) {
          log.trace("Enum: " + clazz.getName() + ", " + value);
          return strToEnum((Class<Enum>) clazz, value.toString());
        } else if (clazz.equals(Class.class)) {
          try {
            return Class.forName(value.toString());
          } catch (ClassNotFoundException e) {
            log.error("Could not find class " + value + ". ", e);
            return null;
          }
        } else {
          // Use string constructor
          try {
            log.trace("String constructor: " + clazz.getName() + ", " + value);
            Constructor<?> constructor;

            // Get wrapper class if necessary
            if (clazz.isPrimitive()) {
              clazz = getWrapperClass(clazz);
            }

            // Support generic Number type by using Double instead (Number doesn't have a string-based constructor).
            if (clazz == Number.class) {
              clazz = Double.class;
            }

            // TODO: handle non-integer string-based construction. String based constructors have been deprecated since
            //       Java 9, and marked for removal, so allowing non-integer numerics to fall through to the
            //       string based construction via constructor.newInstance below will stop working.
            // Make sure to remove decimals/trailing characters (l,L,f,F,d,D) from value before creating target numeric type
            if (clazz == Short.class || clazz == Integer.class || clazz == Long.class) {
              if (isLong(value)) {
                // first remove the "l" or "L" from the value, as Long.valueOf can't handle it
                String longValue = value.toString();
                longValue = value.toString().substring(0, longValue.length() - 1);
                if (clazz.equals(Short.class)) {
                  value = Short.valueOf(longValue);
                } else if (clazz.equals(Integer.class)) {
                  value = Integer.valueOf(longValue);
                } else {
                  value = Long.valueOf(longValue);
                }
              } else if (isNumeric(value)) {
                if (clazz.equals(Short.class)) {
                  value = Double.valueOf(value.toString()).shortValue();
                } else if (clazz.equals(Integer.class)) {
                  value = Double.valueOf(value.toString()).intValue();
                } else {
                  value = Double.valueOf(value.toString()).longValue();
                }
              }
            }

            if (clazz == Symbol.class) {
              output = Factory.createFOL(value.toString());
            } else if (clazz == Variable.class) {
              output = Factory.createVariable(value.toString());
            } else if (clazz == Term.class) {
              //TODO:brad: is this an ok way to do this?
              //brad: it seems like we don't need this cast, but how can we make sure this is a term not a predicate
              //alternatively is there an equivalent of Factory.createPredicate for edu.tufts.hrilab.fol.Term? or is createPredicate sufficient
              output = Factory.createPredicate(value.toString());
            } else if (clazz == Predicate.class) {
              output = Factory.createPredicate(value.toString());
            } else {
              constructor = clazz.getConstructor(String.class);
              output = constructor.newInstance(value.toString());
            }

          } catch (NoSuchMethodException e) {
            log.error("No string based constructor for type: {}. value: {}", clazz.getName(), value, e);
            return null;
          } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
            log.error("Problem instantiating new object of type: {} from {} {}. Returning null.", clazz.getName(), value.getClass(), value, e);
            return null;
          }
        }
      }
    }
    return output;
  }

  /**
   * Checks if a class can be assigned to another class
   *
   * @param fromClazz
   * @param toClazz
   * @return true if clazz can be assigned to toClazz
   */
  public static boolean isAssignable(Class<?> fromClazz, Class<?> toClazz) {
    fromClazz = fromClazz.isPrimitive() ? getWrapperClass(fromClazz) : fromClazz;
    toClazz = toClazz.isPrimitive() ? getWrapperClass(toClazz) : toClazz;

    if (fromClazz != null && toClazz != null) {
      // Check if java supports the assignment
      if (toClazz.isAssignableFrom(fromClazz)) {
        return true;
      } else if (String.class.isAssignableFrom(fromClazz)) {
        // Check if we're converting from a string (we support string constructors (see convertToType))
        return true;
      }
//      else if (!Collection.class.isAssignableFrom(fromClazz) && Symbol.class.isAssignableFrom(toClazz)) {
      else if (Symbol.class.isAssignableFrom(toClazz)) {
        //TODO:brad: I'm not sure the comment below is valid. I added a temprary fix where we don't convert lists into symbols, probably should be all collections?

        // let pretty much everything be converted into a Symbol
        return true;
      } else if (fromClazz.equals(Symbol.class)) {
        // allow conversion from Symbol (not Term/Predicate/Variable) to the following types
        // NOTE: if this is expanding to allow more conversions from Symbol, a similar change
        //       is also likely needed in com.action.Database::filterEntries
        return toClazz.isAssignableFrom(String.class) || Utilities.isNumericType(toClazz) || Utilities.isBooleanType(toClazz);
      }

      // Check if we're dealing with numbers (we support assignments between numbers, see convertToType)
      if (Number.class.isAssignableFrom(fromClazz) && Number.class.isAssignableFrom(toClazz)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Attempt to infer argument types in action scripts when values are used directly in scripts without explicitly
   * defining the argument type.
   *
   * @param value
   * @return
   */
  public static Class<?> getArgumentType(String value) {
    if (isNumeric(value)) {
      if (isInteger(value)) {
        return Integer.class;
      } else if (isLong(value)) {
        return Long.class;
      } else if (isFloat(value)) {
        return Float.class;
      } else {
        return Double.class;
      }
    } else if (isBoolean(value)) {
      return Boolean.class;
    } else if (value.startsWith("\"") && value.endsWith("\"")) {
      return String.class;
    } else {
      // return FOL class in the case of non-numerical, quote-less, non-variable, non-bool value.
      // this might be an unsafe default case, but probably better than returning null
//      return Symbol.class;
      Symbol fol = Factory.createFOL(value);
      if (fol != null) {
        return fol.getClass();
      } else {
        return Symbol.class;
      }
    }
  }

  /**
   * Create FOL instance from an ActionBinding's underlying value. Will return null
   * if the ActionBinding's value is null.
   *
   * TODO: this functionality should probably be moved convertToType
   * @param input
   * @return
   */
  @Deprecated
  public static Symbol createFOL(ActionBinding input) {
    Object val = input.getBindingDeep();
    if (val == null) {
      return null;
    } else if (Symbol.class.isAssignableFrom(val.getClass())) {
      // val is already in the fol class hierarchy
      return (Symbol) val;
    } else if (Justification.class.isAssignableFrom(val.getClass())) {
      //TODO:brad: it could be nice to have a method to get a single predicate directly instead of having to use toString.
      return Factory.createFOL(val.toString());
    } else if (Utilities.isNumericType(val.getClass()) || Utilities.isBooleanType(val.getClass())) {
      // numeric or boolean value
      return Factory.createSymbol(val.toString());
    } else if (String.class.isAssignableFrom(val.getClass())) {
      // Strings
      return Factory.createSymbol(val.toString());
    } else {
      // some non-fol object -- for now all we can do is put its string value in a symbol
      return Factory.createSymbol("\""+val.toString()+"\"");
    }
  }

}
