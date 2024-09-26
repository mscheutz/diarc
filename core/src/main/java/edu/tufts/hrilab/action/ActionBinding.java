/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.util.Utilities;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * An <code>ActionBinding</code> is an association between a variable name and a
 * value. This is really a variable, insofar as it can be uninitialized (i.e.,
 * have no value associated with it). An uninitialized binding consists of a
 * name and a type, and may later be bound to an object.
 */
public class ActionBinding implements Cloneable, Serializable {

  protected final static Logger log = LoggerFactory.getLogger(ActionBinding.class);

  /**
   * The name of this variable.
   */
  public final String name;
  /**
   * Semantic type.
   */
  private final String semanticType;
  /**
   * The Java type of this variable (i.e., Java class).
   */
  public final Class<?> javaType;
  /**
   * This variable's value.
   */
  private Object value;
  /**
   * This variable's default value.
   */
  public final String defaultValue;
  /**
   * Is this ActionBinding a local variable.
   */
  public final boolean isLocal;
  /**
   * If this ActionBinding is a return value.
   */
  public final boolean isReturn;
  /**
   * If this ActionBinding is bound to a value.
   */
  private boolean isBound;
  /**
   * If this ActionBinding represents an argument for a varargs method. Only
   * applies to Java methods (i.e., primitives and opspecs, not scripts).
   */
  private boolean isVarArg;


  /**
   * A new bound variable.
   *
   * @param name the name of this variable
   * @param semanticType the semantic type of this variable
   * @param javaType the type of this variable
   * @param defaultValue the default value of this variable
   * @param value the value to which the variable is bound
   * @param isBound whether value is bound (bound value can be null)
   * @param isReturn whether return values will be bound here
   * @param isVarArg if this represents a varArg argument
   */
  private ActionBinding(String name, String semanticType, Class<?> javaType, String defaultValue, Object value, boolean isBound, boolean isLocal, boolean isReturn, boolean isVarArg) {
    this.name = name;
    this.semanticType = semanticType;
    this.defaultValue = defaultValue;
    this.isLocal = isLocal;
    if (name.charAt(0) == '!' && !isLocal) {
      log.warn("isLocal should be set to true for variable: " + name);
    }
    this.isReturn = isReturn;

    // try to infer the actual java type from the value
    if (javaType.equals(Object.class) && value != null && value.getClass().equals(String.class)) {
      this.javaType = Utilities.getArgumentType((String) value);
    } else {
      this.javaType = javaType;
    }

    this.isBound = isBound; // can be true, even if value is (explicitly) set to null
    if (value != null) {
      if (value instanceof ActionBinding) {
        if (!Utilities.isAssignable(((ActionBinding) value).getBindingTypeDeep(), this.javaType)) {
          ActionBinding binding = (ActionBinding) value;
          log.error("Binding " + name + " with type " + this.javaType + " is not compatible with "
                  + binding.getName() + " of type " + binding.getBindingTypeDeep());
        }
        this.value = value;
      } else {
        this.value = Utilities.convertToType(this.javaType, value);
      }
    } else if (defaultValue != null) {
      // Bind to default value
      // TODO: should this happen here? and should this set isBound==true?
      this.value = Utilities.convertToType(this.javaType, defaultValue);
      this.isBound = true;
    }

    this.isVarArg = isVarArg;
  }

  /**
   * If this binding has a semantic type.
   * @return
   */
  public boolean hasSemanticType() {
    return semanticType != null && !semanticType.isEmpty();
  }

  /**
   * Deep copy an ActionBinding object.
   */
  public ActionBinding(ActionBinding other) {
    isLocal = other.isLocal; //name = new String(name.substring(1));
    name = other.name;
    javaType = other.javaType;
    defaultValue = other.defaultValue;
    isReturn = other.isReturn;
    semanticType = other.semanticType;

    isBound = other.isBound;
    isVarArg = other.isVarArg;

    if(other.value != null) {
      if(other.value instanceof ActionBinding) {
        this.value = new ActionBinding((ActionBinding)other.value);
      } else {
        this.value = other.value;
      }
    }
  }

  public boolean isBound() {
    return isBound;
  }

  public boolean isVarArg() {
    return isVarArg;
  }

  public boolean isReturn() {
    return isReturn;
  }

  public boolean isLocal() {
    return isLocal;
  }

  public String getDefaultValue() {
    return defaultValue;
  }

  /**
   * Bind (or rebind) a variable
   *
   * @param value the value to which the variable is bound
   */
  public void bind(Object value) {
    if (value instanceof ActionBinding) {
      if (Utilities.isAssignable(((ActionBinding) value).getBindingTypeDeep(), javaType)) {
        this.value = value; // (Deep) bind
      } else {
        log.warn("Attempting to bind to an ActionBinding of different type. Converting...");
        this.value = Utilities.convertToType(javaType, ((ActionBinding) value).getBindingDeep());
      }
    } else {
      this.value = Utilities.convertToType(javaType, value);
    }
    isBound = true;
  }

  /**
   * Bind (or rebind) a variable, recursing through bindings to the bottom
   *
   * @param value the value to which the variable is bound
   */
  public void bindDeep(Object value) {
    // if value is an instance of "class binding"
    if (this.value instanceof ActionBinding) {
      // Keep going
      ((ActionBinding) this.value).bindDeep(value);
    } else {
      this.value = Utilities.convertToType(javaType, value);
    }
    isBound = true;
  }

  /**
   * Get the variable's binding value.
   *
   * @return the value to which the variable is bound
   */
  public Object getBinding() {
    return value;
  }

  /**
   * Get the semantic type.
   * @return
   */
  public String getSemanticType() {
    return semanticType;
  }

  /**
   * Get the variable's Java type.
   *
   * @return the type of the variable
   */
  public Class<?> getJavaType() {
    return javaType;
  }

  /**
   * Get the variable's binding, recursing through bindings to the bottom.
   *
   * @return the value to which the variable is bound
   */
  public Object getBindingDeep() {
    if (isVarArg && javaType.isArray()) {
      // if varArg is an array, get deep bindings for any ActionBinding elements.
      // this is the case when individual varArgs have been packaged up into
      // a single ActionBinding for passing into an operator/action
      Object[] varArgs = (Object[]) value;
      List<Object> boundVarArgs = new ArrayList<>();
      Arrays.stream(varArgs).forEach(varArg -> {
        if (varArg instanceof ActionBinding) {
          boundVarArgs.add(((ActionBinding) varArg).getBindingDeep());
        } else {
          boundVarArgs.add(varArg);
        }
      });
      return boundVarArgs.toArray();
    } else if (value instanceof ActionBinding) {
      // If the object to which this is bound is an ActionBinding, recurse
      return Utilities.convertToType(javaType, ((ActionBinding) value).getBindingDeep());
    } else {
      return value;
    }
  }

  /**
   * get the variable's binding name
   *
   * @return the name to which the variable is bound
   */
  public String getName() {
    return name;
  }

  /**
   * get the variable's binding name, recursing through bindings to the bottom
   *
   * @return the name to which the variable is bound
   */
  public String getBindingNameDeep() {
    ActionBinding tmpBinding;
    // If the object to which this is bound is an ActionBinding, recurse
    if (this.getClass().isInstance(value)) {
      tmpBinding = (ActionBinding) value;
      return tmpBinding.getBindingNameDeep();
    } else {
      return name;
    }
  }

  /**
   * Check if binding has a (non-null, non-empty) default value
   * @return true if default value available
   */
  public boolean hasDefaultValue() {
    return defaultValue != null;
  }

  /**
   * get the variable's binding type, recursing through bindings to the bottom
   *
   * @return the type to which the variable is bound
   */
  public Class<?> getBindingTypeDeep() {
    // If the object to which this is bound is an ActionBinding, recurse
    if (this.getClass().isInstance(value)) {
      ActionBinding tmpBinding = (ActionBinding) value;
      return tmpBinding.getBindingTypeDeep();
    } else {
      return javaType;
    }
  }

  @Override
  public ActionBinding clone() {
    try {
      return (ActionBinding) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError();
    }
  }

  @Override
  public String toString() {
    if (value == null) {
      return name + " " + javaType;
    } else if (this.getClass().isInstance(value)) {
      ActionBinding tmpBinding = (ActionBinding) value;
      return name + " " + javaType + " "
              + tmpBinding.name;
    } else {
      return name + " " + javaType + " " + value;
    }
  }

  @Override
  public boolean equals(Object o) {
    if (o == null) {
      return false;
    }
    if (!o.getClass().equals(this.getClass())) {
      return false;
    }
    ActionBinding other = (ActionBinding) o;

    if (!Objects.equals(other.name, this.name)) {
      return false;
    }
    if (!Objects.equals(other.javaType, this.javaType)) {
      return false;
    }
    if (!Objects.equals(other.value, this.value)) {
      return false;
    }
    if (!Objects.equals(other.defaultValue, this.defaultValue)) {
      return false;
    }
    if (other.isLocal != this.isLocal) {
      return false;
    }
    if (other.isReturn != this.isReturn) {
      return false;
    }
    if (other.isBound != this.isBound) {
      return false;
    }
    return true;
  }

  @Override
  public int hashCode() {
    int result = 1;
    result = 31 * result + Objects.hashCode(name);
    result = 31 * result + Objects.hashCode(javaType);
    result = 31 * result + Objects.hashCode(value);
    result = 31 * result + Objects.hashCode(defaultValue);
    result = 31 * result + (isLocal ? 1 : 0);
    result = 31 * result + (isReturn ? 1 : 0);
    return 31 * result + (isBound ? 1 : 0);
  }

  public static class Builder {
    /**
     * The name of this variable.
     */
    private String name;
    /**
     * Semantic type.
     */
    private String semanticType;
    /**
     * The Java type of this variable (i.e., Java class).
     */
    public Class<?> javaType;
    /**
     * This variable's value.
     */
    private Object value = null;
    /**
     * This variable's default value.
     */
    private String defaultValue = null;
    /**
     * If this ActionBinding is a local variable.
     */
    private boolean isLocal = false;
    /**
     * If this ActionBinding is a return value.
     */
    private boolean isReturn = false;
    /**
     * If this ActionBinding is bound to a value.
     */
    private boolean isBound = false;
    /**
     * If this ActionBinding represents an argument for a varargs method. Only
     * applies to Java methods (i.e., primitives and opspecs, not scripts).
     */
    private boolean isVarArg = false;

    /**
     * Instantiate builder. name can optionally define semantic type as well as name (i.e., name:type).
     * @param name binding name and optionally the semantic type
     * @param javaType
     */
    public Builder(String name, Class<?> javaType) {
      // split name into name and type if contains ":"
      if (name.contains(":")) {
        String[] splitName = name.split(":");
        this.name = splitName[0];
        this.semanticType = splitName[1];
      } else {
        this.name = name;
        this.semanticType = "";
      }

      this.javaType = javaType;
    }

    public ActionBinding build() {
      ActionBinding binding = new ActionBinding(name, semanticType, javaType, defaultValue, value, isBound, isLocal, isReturn, isVarArg);
      return binding;
    }

    public Builder setSemanticType(String semanticType) {
      this.semanticType = semanticType;
      return this;
    }

    public Builder setIsLocal(boolean isLocal) {
      this.isLocal = isLocal;
      return this;
    }

    public Builder setIsReturn(boolean isReturn) {
      this.isReturn = isReturn;
      return this;
    }

    public Builder setValue(Object value) {
      this.value = value;
      this.isBound = true;
      return this;
    }

    public Builder setDefaultValue(String value) {
      this.defaultValue = value;
      return this;
    }

    public Builder setIsVarArg(boolean isVarArg) {
      this.isVarArg = isVarArg;
      return this;
    }

  }

}
