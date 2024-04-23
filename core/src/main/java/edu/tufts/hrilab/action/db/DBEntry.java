/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;

import java.io.Serializable;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * TODO: this class (and all sub-classes) are meant to be immutable, but currently several methods
 *   return ActionBindings which are not immutable, which breaks the immutability of this class.
 */
public abstract class DBEntry implements Serializable {

  /**
   * allows for serialization in other locations
   */
  private static final long serialVersionUID = 779232533378720982L;

  /**
   * Each entry in the database has an associated type. The type may be a name
   * of an action or an operator. For example, the type could be "liftup". The
   * type is also used to define a type hierarchy. The action with
   * a type "liftup" may have a parent type of "action". The type hierarchy is
   * used by isA.
   */
  protected final String type;

  /**
   * The description is human-readable text describing the entry. The
   * description is optional and an empty string by default.
   */
  protected final String description;

  /**
   * Roles indicates the list of arguments expected for this entry. Default
   * values may be provided and used when it is invoked if no other value is
   * provided.
   */
  protected final List<ActionBinding> roles;
  
  protected final boolean isVarArgs;

  protected DBEntry(String type, String description, List<ActionBinding> roles, boolean isVarArgs) {
    this.type = type;
    this.description = description;
    this.roles = roles;
    this.isVarArgs = isVarArgs;
  }

  /**
   * Get the type of this entry.
   *
   * @return the type
   */
  public String getType() {
    return this.type;
  }

  /**
   * Get the name of this entry (same as type)
   *
   * @return the name
   */
  public String getName() {
    return this.type;
  }


  /**
   * Get the description
   *
   * @return the description
   */
  public String getDescription() {
    return description;
  }

  /**
   * Get a role's name
   * @param pos the argument position
   * @return that role's type
   */
  public String getRoleName(int pos) {
    return roles.get(pos).name;
  }

  /**
   * Get a role's Java type.
   *
   * @param pos the argument position
   * @return that role's type
   */
  public Class<?> getRoleJavaType(int pos) {
    return roles.get(pos).javaType;
  }

  /**
   * Get a role's semantic type.
   *
   * @param pos the argument position
   * @return that role's semantic type
   */
  public String getRoleSemanticType(int pos) {
    return roles.get(pos).getSemanticType();
  }


  // TODO test these next two
  public String getRoleDefault(int pos) {
    if (roles.get(pos).defaultValue != null) {
      return roles.get(pos).defaultValue.toString();
    } else {
      return null;
    }
  }

  public boolean getRoleReturn(int pos) {
    return roles.get(pos).isReturn;
  }

  public boolean getRoleLocal(int pos) {
    return roles.get(pos).isLocal;
  }

  public boolean getRoleIsVarArg(int pos) {
    return roles.get(pos).isVarArg();
  }
  
  /**
   * Get the number of roles for this entry.
   *
   * @return the number of roles
   */
  public int getNumRoles() {
    return roles.size();
  }

  /**
   * Get roles.
   * @return roles (read only)
   */
  public List<ActionBinding> getRoles() {
    return Collections.unmodifiableList(roles);
  }

  /**
   * Return role with matching role name, and null otherwise.
   * @param name
   * @return
   */
  public ActionBinding getRole(String name) {
    return roles.stream()
            .filter(role -> role.name.equals(name))
            .findFirst()
            .orElse(null);
  }
  
  /**
   * Returns whether the underlying Java method is a varargs method.
   * @return 
   */
  public boolean isVarArgs() {
    return isVarArgs;
  }

  /**
   * Get required roles. The required input roles, excluding the ?actor role.
   * @return required roles (read only)
   */
  public List<ActionBinding> getRequiredInputRoles() {
    return Collections.unmodifiableList(
        roles.stream()
                .filter(r -> (!r.isLocal && !r.isReturn && !r.hasDefaultValue() && !r.name.equals("?actor")))
                .collect(Collectors.toList()));
  }

  /**
   * Get optional roles. This consists of optional input roles that have a default value.
   * @return optional roles (read only)
   */
  public List<ActionBinding> getOptionalInputRoles() {
    return Collections.unmodifiableList(
        roles.stream().filter(r -> (!r.isLocal && r.hasDefaultValue())).collect(Collectors.toList()));
  }

  /**
   * Get the ?actor role (for Actions, not Operators), all required roles, and optional
   * input roles (e.g., those with a default value).
   * This INCLUDES the ?actor role for Actions, not Operators.
   * @return all input roles roles (read only)
   */
  public List<ActionBinding> getInputRoles() {
    return Collections.unmodifiableList(
            roles.stream().filter(r -> (!r.isLocal && !r.isReturn)).collect(Collectors.toList()));
  }

  /**
   * Get the return roles of this DBEntry.
   * @return list of return ActionBindings
   */
  public List<ActionBinding> getReturnRoles() {
    return roles.stream().filter(r -> r.isReturn).collect(Collectors.toList());
  }

  /**
   * Get the local roles of this DBEntry.
   * @return list of return ActionBindings
   */
  public List<ActionBinding> getLocalRoles() {
    return roles.stream().filter(r -> r.isLocal).collect(Collectors.toList());
  }

  /**
   * Get the list of required role types when calling this DBEntry.
   * Required roles are the arguments that have to be provided when
   * calling this DBEntry. This excludes the ?actor role.
   *
   * @return list of role types required when calling this DBEntry.
   */
  public List<Class<?>> getRequiredInputRolesTypes() {
    return getRequiredInputRoles().stream()
            .map(ActionBinding::getJavaType).collect(Collectors.toList());
  }

  /**
   * Get the list of input role types when calling this DBEntry.
   * Input roles are all the arguments that can be provided when
   * calling this DBEntry. This includes the ?actor role, all required roles,
   * and all optional input roles that have a default value.
   * This INCLUDES the ?actor role.
   *
   * @return list of role types (required and optional) when calling this DBEntry.
   */
  public List<Class<?>> getInputRolesTypes() {
    return getInputRoles().stream()
            .map(ActionBinding::getJavaType).collect(Collectors.toList());
  }

  /**
   * Get the list of optional role types when calling this DBEntry.
   * Optional roles are either return variables or roles with default values.
   * They do not have to be provided when calling this DBEntry.
   *
   * @return list of optional role types.
   */
  public List<Class<?>> getOptionalInputRolesTypes() {
    return getOptionalInputRoles().stream()
        .map(ActionBinding::getJavaType).collect(Collectors.toList());
  }

  /**
   * Get the list of return role types of this DBEntry.
   *
   * @return list of return role types
   */
  public List<Class<?>> getReturnRolesTypes() {
    return getReturnRoles().stream()
            .map(ActionBinding::getJavaType).collect(Collectors.toList());
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder("DBEntry\n");

    sb.append("\ttype: " + type + '\n');
    sb.append("\tdescription: " + description + '\n');
    sb.append("\troles:\n");
    for (ActionBinding b : roles) {
      sb.append("\t\t" + b.javaType.getName() + " " + b.name);
      if(b.defaultValue !=null && !b.defaultValue.toString().isEmpty()) {
        sb.append(" - default: " + b.defaultValue.toString());
      }
      if(b.isReturn) {
        sb.append(" - return");
      }
      sb.append('\n');
    }
    return sb.toString();
  }

  /**
   * Checks if another DBEntry conflicts with this one (same type, name and role types).
   * Two DBEntries are conflicting when it is not possible to identify which one is being
   * called in an eventspec, because they have the same type, name and roles.
   * @param that other DBEntry
   * @return true if they conflict
   */
  public boolean conflictsWith(DBEntry that) {
    int typeComparison = this.getClass().toString().compareTo(that.getClass().toString());
    if(typeComparison == 0) {                                                     // Compare type
      int nameComparison = this.getName().compareTo(that.getName());
      if (nameComparison == 0) {                                                  // Compare name
        List<Class<?>> argsA = this.roles.stream().filter(r -> (!r.isLocal && !r.isReturn))
            .map(ActionBinding::getJavaType).collect(Collectors.toList());
        List<Class<?>> argsB = that.roles.stream().filter(r -> (!r.isLocal && !r.isReturn))
            .map(ActionBinding::getJavaType).collect(Collectors.toList());
        if (argsA.size() == argsB.size()) {                                       // Compare number of arguments
          for (int i = 0; i < argsA.size(); i++) {
            if (!argsA.get(i).equals(argsB.get(i))) {
              return false;
            }
          }
          return true;                                                            // Roles identical
        }
      }
    }
    return false;
  }

}
