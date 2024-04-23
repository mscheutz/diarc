/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.util.Utilities;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

public class Domain {
  private static Logger log = LoggerFactory.getLogger(Domain.class);
  private final List<String> requirements = new ArrayList<>();
  private final Map<String, Type> types = new HashMap<>();
  private final Map<String, Predicate> predicates = new HashMap<>(); // predicates hashed by predicate name
  private final List<Action> actions = new ArrayList<>();
  private final List<Action> events = new ArrayList<>();
  private final Map<String, Symbol> constants = new HashMap<>();
  private final Map<String, Predicate> functions = new HashMap<>();
  private final List<Derived> derived = new ArrayList<>();
  private final List<String> supportedRequirements = Arrays.asList("strips", "fluents", "typing", "negative-preconditions", "equality");

  /**
   * Generate a PDDL formatted string for this Domain.
   *
   * @param name
   * @return
   */
  public String generate(String name) {
    StringBuilder prefix = new StringBuilder();
    prefix.append("(define (domain ").append(name).append("domain").append(")\n");
    StringJoiner domain = new StringJoiner("\n", prefix, "\n)");

    //requirements
    StringJoiner reqString = new StringJoiner(" ", "(:requirements ", ")\n");
    for (String req : requirements) {
      reqString.add(":" + req);
    }
    domain.add(reqString.toString());

    //types
    StringJoiner typeString = new StringJoiner("\n", "(:types\n", "\n)\n");
    for (Type type : types.values()) {
      if (!type.getSubTypes().isEmpty()) {
        typeString.add("\t" + type.generate());
      }
    }
    domain.add(typeString.toString());

    //constants
    if(!constants.isEmpty()) {
      StringJoiner constantsString = new StringJoiner("\n", "(:constants\n", "\n)\n");
      for (Symbol constant : constants.values()) {
        constantsString.add("\t" + Generator.generateTyped(constant));
      }
      domain.add(constantsString.toString());
    }

    //predicates
    StringJoiner predString = new StringJoiner("\n", "(:predicates\n", "\n)\n");
    for (Predicate pred : predicates.values()) {
      String generatedPred = Generator.generateTyped(pred);
      if (generatedPred != null) {
        predString.add("\t" + generatedPred);
      } else {
        log.warn("Typed pddl string could not be generated from: " + pred);
      }
    }
    domain.add(predString.toString());

    //functions
    StringJoiner functionString = new StringJoiner("\n", "(:functions\n", "\n)\n");
    for (Predicate func : functions.values()) {
      String generatedFunc = Generator.generateTyped(func);
      if (generatedFunc != null) {
        functionString.add("\t" + generatedFunc);
      } else {
        log.warn("Pddl string could not be generated from: " + func);
      }
    }
    domain.add(functionString.toString());

    //actions
    for (Action action : actions) {
      domain.add(action.generate());
    }

    //events
    for (Action event : events) {
      domain.add(event.generate());
    }

    //derived
    for (Derived d : derived) {
      domain.add(d.generate());
    }

    return domain.toString();
  }

  public List<String> getRequirements() {
    return requirements;
  }

  public List<Type> getTypes() {
    return new ArrayList<>(types.values());
  }

  public Type getType(String typeName) {
    return types.get(typeName);
  }

  public void addRequirement(String req) {
    if (supportedRequirements.contains(req)) {
      this.requirements.add(req);
    } else {
      log.warn("Requirement is not supported: " + req);
    }
  }

  public void addType(Type type) {
    this.types.put(type.getName(), type);
  }

  public void addAction(Action action) {
    for (Action a : this.actions) {
      if (a.getName().equals(action.getName())) {
        log.warn("[addAction] ignoring: " + a + " because it is a duplicate or overload");
        return;
      }
    }
    this.actions.add(action);
  }

  public void addEvent(Action event) {
    for (Action a : this.events) {
      if (a.getName().equals(event.getName())) {
        log.warn("[addEvent] ignoring: " + a + " because it is a duplicate or overload");
        return;
      }
    }
    this.events.add(event);
  }

  /**
   * Return shallow copy of actions.
   *
   * @return
   */
  public List<Action> getActions() {
    return new ArrayList<>(actions);
  }

  public void addPredicate(Predicate predicate) {
    predicate = (Predicate) predicate.toUnnegatedForm();
    if (Utilities.isFullyTyped(predicate)) {
      Predicate existingPred = predicates.get(predicate.getName());
      if (existingPred != null) {
        // NOTE: this assumes new predicate and existing predicate have "matching" forms
        Predicate combined = compareSupers(predicate, existingPred);
        if (combined != null) {
          predicates.put(combined.getName(), combined);
        }
      } else {
        predicates.put(predicate.getName(), predicate);
      }
    }
  }


  public void addDerived(Derived derived) {
    this.derived.add(derived);
  }

  public boolean containsPredicate(Predicate predicate) {
    Predicate matched = predicates.get(predicate.getName());
    if (matched != null) {
      // TODO: this does not check against type hierarchy
      return predicate.instanceOf(matched);
    }
    return false;
  }

  public void addConstant(Symbol constant) {
    if (constants.containsKey(constant.getName())) {
      if (!constants.get(constant.getName()).equals(constant)) {
        log.warn("Constant of same name and different type already exists. Ignoring: " + constant);
      }
    } else {
      constants.put(constant.getName(), constant);
    }
  }

  public boolean containsConstant(Symbol constant) {
    Symbol existingConst = constants.get(constant.getName());
    if (existingConst != null) {
      return constant.equals(existingConst);
    }
    return false;
  }

  //todo: would be good if we could consolidate the name/Symbol comparison stuff all over the pddl code. Symbols don't always have their types.
  public boolean containsConstantNamed(String constantName) {
    Symbol existingConst = constants.get(constantName);
    if (existingConst != null) {
      return constantName.equals(existingConst.getName());
    }
    return false;
  }

  public List<Symbol> getConstants() {
    return new ArrayList<>(constants.values());
  }

  public Symbol getConstant(String constantName) {
    return constants.get(constantName);
  }

  public void addFunction(Predicate function) {
    if (Utilities.isFullyTyped(function)) {
      Predicate existingFunction = functions.get(function.getName());
      if (existingFunction != null) {
        // NOTE: this assumes new functions and existing function have "matching" forms
        Predicate combined = compareSupers(function, existingFunction);
        if (combined != null) {
          functions.put(combined.getName(), combined);
        }
      } else {
        functions.put(function.getName(), function);
      }
    }
  }

  public boolean containsFunction(Symbol function) {
    return functions.containsKey(function.getName());
  }

  public Map<String, Predicate> getFunctions() {
    return new HashMap<>(functions);
  }

  //Combines two iterations of the same predicate into one where the parameters are the highest ontological level
  private Predicate compareSupers(Predicate pred1, Predicate pred2) {
    List<Symbol> newArgs = new ArrayList<>();

    if (pred1.getArgs().size() != pred2.getArgs().size()) {
      log.warn("Overloaded predicate: " + pred1 + " and " + pred2);
      return null;
    }

    log.trace("Comparing: " + pred1 + " : " + pred2);
    for (int i = 0; i < pred1.getArgs().size(); i++) {
      String arg1 = pred1.get(i).getType();
      String arg2 = pred2.get(i).getType();
      String superType = getCommonSuper(arg1, arg2);
      if (superType.equals("unknown")) {
        log.debug("Could not combine: " + pred1 + " + " + pred2);
      }
      newArgs.add(Factory.createVariable("?v" + i, superType));
    }
    Predicate newPred = Factory.createPredicate(pred1.getName(), newArgs);
    log.trace("Result: " + newPred);
    return newPred;
  }

  private String getCommonSuper(String type1, String type2) {
    if (type1.equals(type2)) {
      return type1;
    }
    Type t1 = getType(type1);
    Type t2 = getType(type2);
    if (t1 == null) {
      log.warn("Invalid type: " + type1);
      return "unknown";
    } else if (t2 == null) {
      log.warn("Invalid type: " + type2);
      return "unknown";
    }

    return t1.getCommonSuper(t2);
  }

  private String getSpecific(String type1, String type2) {
    if (type1.equals(type2)) {
      return type1;
    }
    Type t1 = getType(type1);
    Type t2 = getType(type2);
    if (t1 == null) {
      log.warn("Invalid type: " + type1);
      return "unknown";
    } else if (t2 == null) {
      log.warn("Invalid type: " + type2);
      return "unknown";
    }

    return t1.getCommonSuper(t2);
  }
}
