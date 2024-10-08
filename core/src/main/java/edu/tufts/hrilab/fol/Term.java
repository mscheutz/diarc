/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.fol;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.*;

/**
 * Representation of term logical form.
 */
public class Term extends Symbol implements Cloneable, Serializable {
  private static final Logger log = LoggerFactory.getLogger(Term.class);

  protected List<Symbol> args;
  static final long serialVersionUID = -377965330056047941L;
  @Deprecated
  public boolean add(Symbol a) {
    return args.add(a);
  }

  /**
   * Returns the i-th argument, null if the i is out of range.
   * @param i
   * @return
   */
  public Symbol get(int i) {
    if (i < args.size()) {
      return args.get(i);
    } else {
      log.warn("Arg index out of bounds: " + i + " . args size:"+args.size()+" . Logical expression: " + this);
      return null;
    }
  }

  /**
   * Get arguments as a shallow copy.
   * @return
   */
  public List<Symbol> getArgs() {
    return new ArrayList<>(args);
  }

  @Deprecated
  public List<Symbol> getArgsRef() {
    return args;
  }

  /**
   * Gets arguments as a deep copy.
   * @return
   */
  public List<Symbol> getArgsCopy() {
    ObjectOutputStream out = null;
    List<Symbol> newArgs = null;
    try {
      ByteArrayOutputStream b = new ByteArrayOutputStream();
      out = new ObjectOutputStream(b);
      out.writeObject(args);
      out.flush();
      out.close();
      ObjectInputStream in = new ObjectInputStream(
              new ByteArrayInputStream(b.toByteArray()));
      newArgs = (ArrayList<Symbol>) in.readObject();
    } catch (ClassNotFoundException ex) {
      System.err.println("Error copying args: " + ex);
    } catch (IOException ex) {
      System.err.println("Error copying args: " + ex);
    }
    return newArgs;
  }

  /**
   * Get all Variable instances contained in this Term.
   * @return
   */
  public Set<Variable> getVars() {
    Set<Variable> allVars = new HashSet<>();
    for (Symbol s : args) {
      if (s.isVariable()) {
        allVars.add((Variable) s);
      } else if (s.isTerm()) {
        allVars.addAll(((Term) s).getVars());
      }
    }
    return allVars;
  }

  /**
   * Get all Variable instances in the order in which they appear (depth first in cases of nested Terms).
   * @return
   */
  public List<Variable> getOrderedVars() {
    List<Variable> allVars = new ArrayList<>();
    for (Symbol s : args) {
      if (s.isVariable()) {
        allVars.add((Variable) s);
      } else if (s.isTerm()) {
        allVars.addAll(((Term) s).getOrderedVars());
      }
    }
    return allVars;
  }


  /**
   * Get leaves in order in which they appear (depth first in cases of nested Terms).
   * @return
   */
  public List<Symbol> getOrderedLeaves() {
    List<Symbol> allVars = new ArrayList<>();
    for (Symbol s : args) {
      if (!s.hasArgs()) {
        allVars.add(s);
      } else if (s.isTerm()) {
        allVars.addAll(((Term) s).getOrderedLeaves());
      }
    }
    return allVars;
  }

  /**
   * Gets variable bindings from this Term to another. Supports nested terms (and will return those bindings as well).
   * @author luca
   * @param other Term providing the bindings for variables in this term
   * @return bindings map
   */
  public Map<Variable, Symbol> getBindings(Term other) {
    Map<Variable, Symbol> bindings = new HashMap<>();

    if (other.instanceOf(this)) {
      Iterator<Symbol> iThis = this.getArgs().iterator();
      Iterator<Symbol> iOther = other.getArgs().iterator();

      while (iThis.hasNext() && iOther.hasNext()) {
        Symbol argThis = iThis.next();
        Symbol argOther = iOther.next();

        if (argThis instanceof Term && argOther instanceof Term) {
          Map<Variable, Symbol> subBindings = ((Term) argThis).getBindings((Term) argOther);
          for (Variable v : subBindings.keySet()) {
            if (bindings.containsKey(v)) {
              if (!bindings.get(v).equals(subBindings.get(v))) {
                log.error("Inconsistent bindings between " + this + " and " + other + ": "
                        + v + " is bound to different values " + bindings.get(v) + " and " + subBindings.get(v));
                log.error("Keeping the first encountered value for " + v + ":" + bindings.get(v));
                subBindings.remove(v);
              }
            }
          }
          bindings.putAll(subBindings);
        } else if (argThis instanceof Variable) {
          // TODO: Here, we would ideally check that argOther fits within argThis (type checking)
          if (argOther instanceof Variable && ((Variable) argOther).getValue() != null) {
            bindings.put((Variable) argThis, ((Variable) argOther).getValue()); // Not totally sure about this.
          } else {
            bindings.put((Variable) argThis, argOther);
          }
        } else {
          // Nothing to do here, just a sanity check (instanceOf)
          if (!argThis.equals(argOther)) {
            //fixme: this throws issues if one is typed and the other isn't.
            log.error("Inconsistent symbols between " + this + " and " + other + ": "
                    + "(" + argThis.getClass() + ") " + argThis + " != (" + argThis.getClass() + ") " + argOther
                    + ". This should not happen!");
          }
        }
      }
    }
    return bindings;
  }

  /**
   * Compares "instantiated" term to a template.
   *
   * TODO: FIXME: this is broken when a variable appears more than once in the template and
   * different values are used for that variable
   *
   * @param template
   * @return true if this predicate matches the template.
   */
  public boolean instanceOf(Term template) {
    // used to ensure that when a variable appears more than once in the template, all values are the same
    Map<Variable, Symbol> bindings = new HashMap<>();
    return instanceOfHelper(template, bindings);
  }

  private boolean instanceOfHelper(Term template, Map<Variable, Symbol> bindings) {
    if (this.getName().equals(template.getName())) {
      if (this.size() == template.size()) {
        Iterator<Symbol> iThis = this.getArgs().iterator();
        Iterator<Symbol> iTemplate = template.getArgs().iterator();

        while (iThis.hasNext() && iTemplate.hasNext()) {
          Symbol argThis = iThis.next();
          Symbol argTemplate = iTemplate.next();

          if (argTemplate.isTerm()) {
            if (argThis.isTerm()) {
              if (!(((Term) argThis).instanceOfHelper((Term) argTemplate, bindings))) {
                return false;
              }
            } else {
              return false;
            }
          } else if (argTemplate.isVariable()) {
            if (bindings.containsKey(argTemplate) && !bindings.get(argTemplate).equals(argThis)) {
              return false;
            }
            if (!(argThis.isVariable())) {
              // TODO: Here, we would ideally check that argThis fits within argTemplate (type checking)
              //EW: tentativeAccept acknowledgement colliding with error response falls into this block, are error
              //      response args supposed to be variables? Resolve above TODO
              // TODO: EAK: i don't know why this hack was needed, but it breaks all sorts of things
              //        Variables do not have to start with ?
//              if (!argTemplate.name.startsWith("?")) {
//                return false;
//              }
            }
            bindings.put((Variable) argTemplate, argThis);
          } else {
            if (!argThis.getName().equalsIgnoreCase(argTemplate.getName())) {
              // TODO: Ideally we'd use equals() to compare both Symbols...
              return false;
            }
          }
        }
        return true;
      }
    }
    return false;
  }

  /**
   * Similar to applyBindingMap, except this Term is not bound. Instead,
   * a cloned version is bound and returned.
   * @param bindings
   */
  public Term copyWithNewBindings(Map<Variable, ? extends Symbol> bindings) {
    Term t = this.clone();
    t.applyBindingMap(bindings);
    return t;
  }

  public Set<Term> copyWithNewBindings(List<? extends Map<Variable, ? extends Symbol>> bindings) {
    Set<Term> results = new HashSet<>();
    bindings.forEach(binding -> results.add(this.copyWithNewBindings(binding)));
    return results;
  }

  /**
   * Protected helper method for copyWithNewBindings. Replaces Variables with Symbols from the bindings map.
   * Note that not all Variables need to be bound.
   * @param bindings
   */
  protected void applyBindingMap(Map<Variable, ? extends Symbol> bindings) {
    for (int i = 0; i < size(); ++i) {
      Symbol arg = args.get(i);
      if (arg.isVariable()) {
        Variable arg_var = (Variable) arg;
        if (bindings.containsKey(arg_var)) {
          args.set(i, bindings.get(arg_var));
        }
      } else if (arg.isTerm()) {
        ((Term) arg).applyBindingMap(bindings);
      }
    }
  }


  /**
   * Returns a clone with replaced variables' types with passed in variables' types.
   * @param typedVariables
   * @return
   */
  public Term copyWithNewVariableTypes(Collection<Variable> typedVariables) {
    Term t = this.clone();
    t.applyVariableTypes(typedVariables);
    return t;
  }

  /**
   * Replaces variables' types with passed in variables' types.
   */
  protected void applyVariableTypes(Collection<Variable> typedVariables) {
    for (Symbol s : this.getArgs()) {
      if (s.isVariable()) {
        for (Variable v : typedVariables) {
          if (v.getName().equals(s.getName())) {
            ((Variable) s).setType(v.getType());
            break;
          }
        }
      } else if (s.isTerm()) {
        ((Term) s).applyVariableTypes(typedVariables);
      }
    }
  }

  @Deprecated
  public void set(int i, Symbol newArg) {
    args.set(i, newArg);
  }

  /**
   * Return the number of arguments.
   * @return
   */
  public int size() {
    return args.size();
  }

  /**
   * Compare without checking semantic type information.
   * @param o
   * @return
   */
  public boolean equalsIgnoreType(Object o) {
    // needs to be the same class to be equal
    if (!getClass().equals(o.getClass())) {
      return false;
    }
    Term term = (Term) o;

    // matching on name
    if (!getName().equals(term.getName())) {
      return false;
    }
    // matching on type (to match hashCode, but should always be empty for Term/Predicate)
    if (!getName().equals(term.getName())) {
      return false;
    }
    //match args (order of args matters)
    final List<Symbol> other_args = term.getArgs();
    int size = args.size();
    if (args.size() != other_args.size()) {
      return false;
    }
    for (int i = 0; i < size; ++i) {
      if (!args.get(i).equalsIgnoreType(other_args.get(i))) {
        return false;
      }
    }

    return true;
  }

  /**
   * Check whether the given object is equal to this one. Two Symbols are equal
   * if they're of the same class, their names are the same, and their args are
   * the same.
   *
   * @param o the Object to compare this with
   * @return true if they're equal, false otherwise
   */
  @Override
  public boolean equals(Object o) {
    // needs to be the same class to be equal
    if (!getClass().equals(o.getClass())) {
      return false;
    }
    // matching on name
    if (!getName().equals(((Term) o).getName())) {
      return false;
    }
    //match args (order of args matters)
    final List<Symbol> other_args = ((Term) o).getArgs();
    int size = args.size();
    if (args.size() != other_args.size()) {
      return false;
    }
    for (int i = 0; i < size; ++i) {
      if (!args.get(i).equals(other_args.get(i))) {
        return false;
      }
    }

    return true;
  }

  /**
   * Calculate hash code for this Term.
   *
   * @return the hash code
   */
  @Override
  public int hashCode() {
    final int prime = 31;
    int result = prime + super.hashCode();  //"name" hashCode
    result = result * prime + args.hashCode(); //arg hashCodes

    return result;
  }

  /**
   * Constructs a String representation of the Term. If the term is negated,
   * then it is wrapped in a not().
   *
   * @return String of the form name(args...) or not(name(args...))
   */
  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append(super.toString()).append('(');
    String sp = "";
    for (Symbol a : args) {
      sb.append(sp).append(a.toString());
      sp = ",";
    }
    sb.append(")");
    return sb.toString();
  }

  @Override
  public String toUntypedString() {
    StringBuilder sb = new StringBuilder();
    sb.append(super.toUntypedString()).append('(');
    String sp = "";
    for (Symbol a : args) {
      sb.append(sp).append(a.toUntypedString());
      sp = ",";
    }
    sb.append(")");
    return sb.toString();
  }

  @Override
  public Term toUntyped() {
    List<Symbol> untypedArgs = new ArrayList<>();
    args.forEach(arg -> untypedArgs.add(arg.toUntyped()));
    return new Term(name, untypedArgs);
  }

  @Override
  public Term toUnnegatedForm() {
    if (name.equals("not")) {
      if (args.size() != 1) {
        log.error("Cannot convert to unnegated form with more than one arg. Returning negated form.");
        return this;
      } else if (!args.get(0).isTerm()) {
        log.error("Inner arg is not Term or Predicate. Returning negated form.");
        return this;
      } else {
        return (Term) args.get(0);
      }
    } else {
      // already un-negated
      return this;
    }
  }

  public Term toNegatedForm() {
    if (name.equals("not")) {
      // already negated
      return this.clone();
    } else {
      return new Term("not", this.clone());
    }
  }

  /**
   * Check if this Term is negated (i.e., functor name is "not").
   * @return
   */
  public boolean isNegated() {
    return this.name.equals("not");
  }

  public Term(Term t) {
    super(t.getName(),t.getType());
    args = new ArrayList<>();
    for (Symbol a : t.args) {
      args.add(a.clone());
    }
  }

  public Term(String n, List<? extends Symbol> a) {
    super(n,""); //TODO:brad: what does it mean for a Term to have a type
    addConstructorArgs(a);
  }

  public Term(String n, Symbol... a) {
    super(n,"");//TODO:brad: what does it mean for a Term to have a type
    addConstructorArgs(Arrays.asList(a));
  }

  public Term(Symbol n, List<? extends Symbol> a) {
    super(n);
    addConstructorArgs(a);
  }

  public Term(Symbol n, Symbol... a) {
    super(n);
    addConstructorArgs(Arrays.asList(a));
  }

  public Term(String n, String... a) {
    super(n);
    args = new ArrayList<>();
    for (String s : a) {
      int ind = s.indexOf(":");
      if (ind < 0) {
        if (Character.isUpperCase(s.charAt(0)) || s.startsWith("?") || s.startsWith("!")) {
          args.add(new Variable(s));
        } else {
          args.add(new Symbol(s));
        }
      } else {
        String aname = s.substring(0, ind);
        String atype = s.substring(ind + 1);
        if (Character.isUpperCase(aname.charAt(0)) || aname.startsWith("?") || aname.startsWith("!")) {
          args.add(new Variable(aname, atype));
        } else {
          args.add(new Constant(aname, atype));
        }
      }
    }
  }

  @Override
  public Term clone() {
    Term clone = (Term) super.clone();
    clone.args = new ArrayList<>();
    for (Symbol arg : args) {
      clone.args.add(arg.clone());
    }
    return clone;
  }

  private void addConstructorArgs(List<? extends Symbol> a) {
    args = new ArrayList<>();
    for (Symbol arg : a) {
      args.add(arg.clone());
    }
  }

  /**
   * Checks if this symbol has arguments.
   *
   * @return true if Term has arguments.
   */
  public boolean hasArgs() {
    return !args.isEmpty();
  }

  @Override
  public boolean isTerm() {
    return true;
  }
}
