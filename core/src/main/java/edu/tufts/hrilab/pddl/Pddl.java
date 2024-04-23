/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// This is the highest level PDDL class. It consists of a domain and a problem, and holds the PDDL builder
public class Pddl {

  private final Domain domain;
  private final Problem problem;
  private final String name;
  private static final Logger log = LoggerFactory.getLogger(Pddl.class);

  //todo: Add save to file
  public Pddl(PddlBuilder builder) {
    this.name = builder.name;
    this.domain = builder.domain;
    this.problem = builder.problem;
  }

  public void reset() {
    //todo
  }

  public Domain getDomain() {
    return domain;
  }

  public Problem getProblem() {
    return problem;
  }

  private File writeFile(String filename, String pddlString) {
    File file;
    try {
      file = File.createTempFile(filename, ".pddl");
      FileWriter writer = new FileWriter(file);
      writer.write(pddlString);
      writer.close();
      log.debug("Wrote PDDL file: " + file.getAbsolutePath());
    } catch (IOException e) {
      log.error("Failed to make plan, could not create file.", e);
      return null;
    }
    return file;
  }

  public File generateDomainFile() {
    return writeFile(this.name + "Domain", domain.generate(this.name));
  }

  public File generateProblemFile() {
    return writeFile(this.name + "Problem", problem.generate(this.name));
  }

  public static class PddlBuilder {
    private final String name;
    private final Domain domain;
    private final Problem problem;

    public PddlBuilder(String name) {
      this.name = name;
      this.domain = new Domain();
      this.problem = new Problem();
    }

    public Domain getDomain() {
      return domain;
    }

    public Problem getProblem() {
      return problem;
    }

    public PddlBuilder addRequirement(String req) {
      domain.addRequirement(req);
      return this;
    }

    public PddlBuilder addType(String typeName, String... superTypeNames) {
      //get supertypes
      List<Type> superTypes = new ArrayList<>();
      for (String superTypeName : superTypeNames) {
        Type superType = domain.getType(superTypeName);

        // create superType if it doesn't exist
        if (superType == null) {
          superType = new Type(superTypeName);
          domain.addType(superType);
        }
        superTypes.add(superType);
      }

      Type type = domain.getType(typeName);
      if (type == null) {
        type = new Type(typeName);
        type.addSuperTypes(superTypes);
        domain.addType(type);
      } else {
        type.addSuperTypes(superTypes);
      }
      return this;
    }


    public PddlBuilder addPredicate(String name, List<Symbol> types) {
      List<Symbol> typedArgs = new ArrayList<>();
      for (int i = 0; i < types.size(); ++i) {
        typedArgs.add(Factory.createVariable("?v" + i, types.get(i).getName()));
      }
      Predicate predicate = Factory.createPredicate(name, typedArgs);

      domain.addPredicate(predicate);
      return this;
    }

    public PddlBuilder addPredicate(Predicate predicate) {
      if (predicate.getName().startsWith("and")) {
        for (Symbol s : predicate.getArgs()) {
          if (s.isPredicate()) {
            addPredicate((Predicate) s);
          }
        }
      } else {
        // don't add operators as pddl predicates
        if (!Operators.DIARC_TO_PDDL.containsKey(predicate.toUnnegatedForm().getName())) {
          domain.addPredicate(predicate);
        }
      }
      return this;
    }

    public PddlBuilder addFunction(String name, List<Symbol> types) {
      List<Symbol> typedArgs = new ArrayList<>();
      for (int i = 0; i < types.size(); ++i) {
        typedArgs.add(Factory.createVariable("?v" + i, types.get(i).getName()));
      }
      Predicate function = Factory.createPredicate(name, typedArgs);

      domain.addFunction(function);
      return this;
    }

    public PddlBuilder addAction(Action action) {
      domain.addAction(action);
      return this;
    }

    public PddlBuilder addEvent(Action event) {
      domain.addEvent(event);
      return this;
    }


    public PddlBuilder addObject(String name, String type) {
      problem.addObject(Factory.createSymbol(name, type));
      return this;
    }

    public PddlBuilder addConstant(String name, String type) {
      domain.addConstant(Factory.createSymbol(name, type));
      return this;
    }

    public PddlBuilder addInit(Predicate predicate) {
      if (predicate.getName().startsWith("and")) {
        for (Symbol s : predicate.getArgs()) {
          if (s.isPredicate()) {
            addInit((Predicate) s);
          }
        }
      } else if (domain.containsPredicate(predicate) || predicate.getName().equals("equals")) {
          boolean valid = true;
          for (Symbol arg : predicate.getArgs()) {
            //todo: I'm sure this could be refactored to be a _lot_ more readable and maintainable.
              if (!problem.containsObjectName(arg.getName()) && !domain.containsConstantNamed(arg.getName())) {
                if (!predicate.getName().equals("property_of") && (predicate.getArgs().size() < 2 || !predicate.getArgs().get(1).equals(arg))) {
                  log.debug("[addInit] ignoring fact " + predicate.toUntypedString() + " in state. Some objects in grounding missing from domain");
                  valid = false;
                  break;
                }
              }
          }
          if (valid) {
        problem.addInit(predicate);
          }
      } else if (predicate.getName().equals("fluent_equals")) {
        // filter out fluent_equals states that don't have valid function as first arg
        // should be of form: (fluent_)equals(function, value)
        if (domain.containsFunction(predicate.get(0))) {
          problem.addInit(predicate);
        }
      }
      return this;
    }

    public PddlBuilder addGoal(Predicate predicate) {
      problem.addGoal(predicate);
      return this;
    }

    public PddlBuilder addMetric(Predicate metric) {
      problem.addMetric(metric);
      return this;
    }

    //fixme: This is VERY fragile in how we handle naming the free variables. Consult Marlow if having issues!
    public PddlBuilder addDerived(Predicate head, List<Predicate> body) {
      List<Predicate> typedPreds = new ArrayList<>();
      for (Predicate p : body) {
        typedPreds.add(createTypedPred(p));
      }
      domain.addPredicate(createTypedPred(head));
      domain.addDerived(new Derived(createTypedPred(head), typedPreds));
      return this;
    }

    public Pddl build() {
      Pddl pddl = new Pddl(this);
      validatePddl(pddl);
      return pddl;
    }

    private void validatePddl(Pddl pddl) {

    }

    private Predicate createTypedPred(Predicate p) {
      List<Symbol> typedArgs = new ArrayList<>();
      //todo: this should probably be only for fluent_equals
      if (Operators.DIARC_TO_PDDL.containsKey(p.getName())) {
        Predicate innerPred = createTypedPred((Predicate) p.get(0));
        return Factory.createPredicate(p.getName(), innerPred, p.get(1));
      }
      for (int i = 0; i < p.getArgs().size(); ++i) {
        String symbolName = p.getArgs().get(i).getName();
        if (symbolName.contains("'")) {
          typedArgs.add(Factory.createSymbol(symbolName.replace("'", "")));
        } else {
          typedArgs.add(Factory.createVariable("?v" + i, symbolName));
        }
      }
      return (Factory.createPredicate(p.getName(), typedArgs));
    }
  }
}
