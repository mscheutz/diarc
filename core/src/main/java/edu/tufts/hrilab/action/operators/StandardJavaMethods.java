/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.operators;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import org.apache.commons.lang3.ClassUtils;
import org.apache.commons.lang3.reflect.ConstructorUtils;
import org.apache.commons.lang3.reflect.MethodUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @author Evan Krause
 */
public class StandardJavaMethods {

  private static Logger log = LoggerFactory.getLogger(StandardJavaMethods.class);
  
  /**
   * Operator to instantiate a new Java object of the specified class with a
   * variable number of arguments.
   * @param clazz class to instantiate
   * @param args constructor arguments
   * @return
   * @throws NoSuchMethodException
   * @throws InstantiationException
   * @throws IllegalAccessException
   * @throws InvocationTargetException
   * @throws Throwable 
   */
  public static Object newObject(Class<?> clazz, Object... args) throws NoSuchMethodException, InstantiationException, IllegalAccessException, InvocationTargetException, Throwable {
    Constructor<?> constructor = getConstructor(clazz, args);
    MethodHandle methodHandle = MethodHandles.lookup().unreflectConstructor(constructor);
    return methodHandle.invokeWithArguments(args);
  }

  /**
   * Operator to invoke a method on an instantiated Java object with
   * specified arguments. The invoked method must not be of type void.
   * @param o Java object to invoke method on
   * @param name method name
   * @param args method arguments
   * @return method's return value
   * @throws NoSuchMethodException
   * @throws IllegalAccessException
   * @throws InvocationTargetException
   * @throws Throwable 
   */
  public static Object invokeMethod(Object o, String name, Object... args) throws NoSuchMethodException, IllegalAccessException, InvocationTargetException, Throwable {
    Method method = getMethod(o.getClass(), name, args);

    if (method.isVarArgs()) {
      // TODO: EAK: understand why this doesn't work
//    MethodHandle methodHandle = MethodHandles.lookup().unreflect(method);
//    methodHandle = methodHandle.bindTo(o);
//    methodHandle.invokeWithArguments(args);

      MethodHandle methodHandle = MethodHandles.lookup().bind(o, name, MethodType.methodType(method.getReturnType(), method.getParameterTypes()));
      return methodHandle.invokeWithArguments(args);
    } else {
      return method.invoke(o, args);
    }
  }

  /**
   * Operator to invoke a static Java method with specified arguments. 
   * The invoked method must not be of type void.
   * 
   * @param clazz class containing target static method
   * @param name method name
   * @param args method arguments
   * @return method's return value
   * @throws NoSuchMethodException
   * @throws IllegalAccessException
   * @throws InvocationTargetException
   * @throws Throwable 
   */
  public static Object invokeStaticMethod(Class<?> clazz, String name, Object... args) throws NoSuchMethodException, IllegalAccessException, InvocationTargetException, Throwable {
    Method method = getMethod(clazz, name, args);
    if (method == null) {
      log.error("No static method found for: " + clazz + "." + name + " args: " + args);
      return null;
    }
    MethodHandle methodHandle = MethodHandles.lookup().unreflect(method);
    return methodHandle.invokeWithArguments(args);
  }

  /**
   * Helper method that can handle primitive/wrapped type mismatch as well as
   * varArg constructors.
   *
   * @param clazz
   * @param args
   * @return Constructor
   * @throws NoSuchMethodException
   */
  private static Constructor<?> getConstructor(Class<?> clazz, Object... args) throws NoSuchMethodException {
    // collect arg types
    Class<?>[] argTypes = new Class<?>[args.length];
    for (int i = 0; i < args.length; ++i) {
      argTypes[i] = args[i].getClass();
    }

    // using ConstructorUtils lookup to handle primitive to boxed type matching
    Constructor<?> constructor = ConstructorUtils.getMatchingAccessibleConstructor(clazz, argTypes);

    // if couldn't find direct match, look for potential varArgs match
    if (constructor == null) {
      Constructor<?>[] constructors = clazz.getConstructors();
      for (int i = 0; i < constructors.length; ++i) {
        Constructor<?> currConstructor = constructors[i];

        // check constructor is actually varArgs
        if (!currConstructor.isVarArgs()) {
          continue;
        }

        // check for a sane number of args
        int numParams = currConstructor.getParameterCount();
        int numArgs = argTypes.length;
        if (numArgs < numParams) {
          continue;
        }

        boolean isMatch = true; // assumed innocent until proven guilty
        Class<?>[] paramTypes = currConstructor.getParameterTypes();
        int numConcreteParams = numParams - 1;
        // check that concrete args (i.e., non-varArgs) match param types
        for (int j = 0; j < numConcreteParams; ++j) {
          Class<?> paramType = paramTypes[j];
          Class<?> argType = argTypes[j];
          if (!ClassUtils.isAssignable(argType, paramType, true)) {
            isMatch = false;
            break;
          }
        }

        if (!isMatch) {
          continue;
        }

        // check that rest of args match varArg component type (e.g., String is component type of String[])
        Class<?> varArgComponentType = paramTypes[numConcreteParams].getComponentType();
        for (int j = numConcreteParams; j < numArgs; ++j) {
          Class<?> argType = argTypes[j];
          if (!ClassUtils.isAssignable(argType, varArgComponentType, true)) {
            isMatch = false;
            break;
          }
        }

        if (isMatch) {
          if (constructor != null) {
            throw new NoSuchMethodException("ambiguous: " + constructor + " and " + currConstructor + " both match.");
          }
          constructor = currConstructor;
        }
      }
    }

    if (constructor == null) {
      throw new NoSuchMethodException("No constructor found for " + clazz.toString() + " with args: " + Arrays.toString(args));
    }

    return constructor;
  }

  /**
   * Helper method that can handle primitive/wrapped type mismatch as well as
   * varArg methods.
   *
   * @param clazz
   * @param args
   * @return Method
   * @throws NoSuchMethodException
   */
  private static Method getMethod(Class<?> clazz, String name, Object... args) throws NoSuchMethodException {
    // collect arg types
    Class<?>[] argTypes = new Class<?>[args.length];
    for (int i = 0; i < args.length; ++i) {
      argTypes[i] = args[i].getClass();
    }

    // using MethodUtils lookup to handle primitive to boxed type matching
    Method method = MethodUtils.getMatchingAccessibleMethod(clazz, name, argTypes);

    // if couldn't find direct match, look for potential varArgs match
    if (method == null) {
      Method[] methods = clazz.getMethods();
      for (int i = 0; i < methods.length; ++i) {
        Method currMethod = methods[i];

        // check method has correct name and is actually varArgs
        if (!currMethod.getName().equals(name) || !currMethod.isVarArgs()) {
          continue;
        }

        // check for a sane number of args
        int numParams = currMethod.getParameterCount();
        int numArgs = argTypes.length;
        if (numArgs < numParams) {
          continue;
        }

        boolean isMatch = true; // assumed innocent until proven guilty
        Class<?>[] paramTypes = currMethod.getParameterTypes();
        int numConcreteParams = numParams - 1;
        // check that concrete args (i.e., non-varArgs) match param types
        for (int j = 0; j < numConcreteParams; ++j) {
          Class<?> paramType = paramTypes[j];
          Class<?> argType = argTypes[j];
          if (!ClassUtils.isAssignable(argType, paramType, true)) {
            isMatch = false;
            break;
          }
        }

        if (!isMatch) {
          continue;
        }

        // check that rest of args match varArg component type (e.g., String is component type of String[])
        Class<?> varArgComponentType = paramTypes[numConcreteParams].getComponentType();
        for (int j = numConcreteParams; j < numArgs; ++j) {
          Class<?> argType = argTypes[j];
          if (!ClassUtils.isAssignable(argType, varArgComponentType, true)) {
            isMatch = false;
            break;
          }
        }

        if (isMatch) {
          if (method != null) {
            throw new NoSuchMethodException("ambiguous: " + method + " and " + currMethod + " both match.");
          }
          method = currMethod;
        }
      }
    }

    if (method == null) {
      throw new NoSuchMethodException("No method found for " + clazz + " method: " + name + Arrays.toString(args));
    }

    return method;
  }
}
