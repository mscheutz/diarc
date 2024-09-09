/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner.pddl;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.pddl.Operators;
import fr.uga.pddl4j.parser.Connective;
import fr.uga.pddl4j.parser.Domain;
import fr.uga.pddl4j.parser.Exp;
import fr.uga.pddl4j.parser.NamedTypedList;
import fr.uga.pddl4j.parser.Op;
import fr.uga.pddl4j.parser.Parser;
import fr.uga.pddl4j.parser.Problem;
import fr.uga.pddl4j.parser.Symbol;
import fr.uga.pddl4j.parser.TypedSymbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * This class parses a PDDL domain and problem file and populates several DIARC components with the parsed
 * information. This can include ASL (action scripts) and beliefs (rules, facts). There is not guaranteed
 * support for all valid PDDL features, so it's possible that not all the PDDL will be retained.
 */
public class PddlParser {
  private static Logger log = LoggerFactory.getLogger(PddlParser.class);

  private Set<Predicate> facts = new HashSet<>();
  private Set<ActionDBEntry.Builder> actions = new HashSet<>();
  private Predicate goal;

  public Set<Predicate> getFacts() {
    return facts;
  }

  public Set<ActionDBEntry.Builder> getUnbuiltActions() {
    return actions;
  }

  public Predicate getGoalPredicate() {
    return goal;
  }

  public void parse(String domainFile, String problemFile) {
    Parser parser = new Parser();
    parser.parseFromStream(getClass().getResourceAsStream(domainFile), getClass().getResourceAsStream(problemFile));
    if (!parser.getErrorManager().isEmpty()) {
      StringBuilder errorMsgs = new StringBuilder();
      parser.getErrorManager().getMessages().forEach(message -> errorMsgs.append(message.toString()).append("\n"));
      log.error("Error parsing PDDL: " + errorMsgs);
      return;
    }


    parseDomain(parser.getDomain());
    parseProblem(parser.getProblem());
  }

  private void parseDomain(Domain domain) {
    // :types - create diarc type hierarchy
    // e.g., subtype(var, object)
    List<Predicate> diarcTypes = new ArrayList<>();
    for (TypedSymbol pddlType : domain.getTypes()) {
      String pddlTypeStr = pddlType.getImage();
      for (Symbol pddlSuperType : pddlType.getTypes()) {
        Predicate diarcType = Factory.createPredicate("subtype", pddlTypeStr, pddlSuperType.getImage());
        diarcTypes.add(diarcType);
      }
    }
    facts.addAll(diarcTypes);

    // :predicates - create diarc predicates
    // e.g., holding(?v0 - object) --> predicate(holding, object)
    List<Predicate> diarcPredicates = new ArrayList<>();
    for (NamedTypedList pddlPred : domain.getPredicates()) {
      List<String> args = new ArrayList<>();
      args.add(pddlPred.getName().getImage()); // predicate name
      pddlPred.getArguments().forEach(arg -> args.add(arg.getTypes().get(0).getImage())); // arg types
      Predicate diarcPred = Factory.createPredicate("predicate", args.toArray(new String[args.size()]));
      diarcPredicates.add(diarcPred);
    }
    facts.addAll(diarcPredicates);

    // :functions - create diarc functions
    // e.g., (inventory ?v0 - actor ?v1 - object) --> function(inventory actor object)
    List<Predicate> diarcFunctions = new ArrayList<>();
    for (NamedTypedList pddlFunction : domain.getFunctions()) {
      List<String> args = new ArrayList<>();
      args.add(pddlFunction.getName().getImage()); // function name
      pddlFunction.getArguments().forEach(arg -> args.add(arg.getTypes().get(0).getImage())); // arg types
      Predicate diarcFunction = Factory.createPredicate("function", args.toArray(new String[args.size()]));
      diarcFunctions.add(diarcFunction);
    }
    facts.addAll(diarcFunctions);

    // :constants - create diarc constants
    List<Predicate> diarcConstants = new ArrayList<>();
    for (TypedSymbol pddlConstant : domain.getConstants()) {
      String constant = pddlConstant.getImage();
      for (Symbol pddlType : pddlConstant.getTypes()) {
        Predicate diarcConstant = Factory.createPredicate("constant", constant, pddlType.getImage());
        diarcConstants.add(diarcConstant);
      }
    }
    facts.addAll(diarcConstants);

    // :action - create diarc actions
    for (Op action : domain.getOperators()) {
      String name = action.getName().getImage();
      ActionDBEntry.Builder actionBuilder = new ActionDBEntry.Builder(name);

      // parameters
      for (TypedSymbol param : action.getParameters()) {
        ActionBinding.Builder roleBuilder = new ActionBinding.Builder(param.getImage(), edu.tufts.hrilab.fol.Symbol.class);
        if (!param.getTypes().isEmpty()) {
          roleBuilder.setSemanticType(param.getTypes().get(0).getImage());
        }
        if (actionBuilder.getRoles().isEmpty() && !param.getImage().equals("?actor")) {
          log.warn("Action's first parameter is not ?actor. This will automatically be added as the first parameter. Action: " + name);
        }
        actionBuilder.addRole(roleBuilder.build());
      }

      // pre-conditions
      // TODO: how to set observable?
      for (Exp pddlCond : action.getPreconditions().getChildren()) {
        parseCondition(pddlCond, actionBuilder);
      }

      // post-conditions
      for (Exp pddlEffect : action.getEffects().getChildren()) {
        parseEffect(pddlEffect, actionBuilder);
      }

      // default action step
      List<String> primitiveActionArgs = new ArrayList<>();
      actionBuilder.getRoles().forEach(role -> primitiveActionArgs.add(role.getName()));
      Predicate primitiveAction = Factory.createPredicate(name, primitiveActionArgs.toArray(new String[primitiveActionArgs.size()]));

      // Add operator to instantiate predicate --> op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "actionName(args)");
      ActionBinding primitiveActionRole = new ActionBinding.Builder("!primitiveAction", Predicate.class).setIsLocal(true).build();
      actionBuilder.addRole(primitiveActionRole);
      EventSpec.Builder opBuilder = new EventSpec.Builder(EventSpec.EventType.OPERATOR);
      opBuilder.setCommand("invokeStaticMethod");
      opBuilder.addInputArg("\"edu.tufts.hrilab.fol.Factory\"");
      opBuilder.addInputArg("\"createPredicate\"");
      opBuilder.addInputArg("\"" + primitiveAction + "\"");
      opBuilder.addReturnArg(primitiveActionRole.getName());
      actionBuilder.addEventSpec(opBuilder.build());

      EventSpec.Builder opBuilder2 = new EventSpec.Builder(EventSpec.EventType.OPERATOR);
      opBuilder2.setCommand("executePrimitiveAction");
      opBuilder2.addInputArg(primitiveActionRole.getName());
      actionBuilder.addEventSpec(opBuilder2.build());

      // add un-built action
      actions.add(actionBuilder);
    }

    // TODO: how to handle pddl events?
  }

  private void parseProblem(Problem problem) {
    // :init = create diarc initial states
    List<Predicate> diarcInits = new ArrayList<>();
    for (Exp init : problem.getInit()) {
      diarcInits.add(generateDiarcPredicate(init));
    }
    facts.addAll(diarcInits);

    // :objects = create diarc objects
    List<Predicate> diarcObjects = new ArrayList<>();
    for (TypedSymbol pddlObject : problem.getObjects()) {
      String objectStr = pddlObject.getImage();
      for (Symbol pddlType : pddlObject.getTypes()) {
        Predicate diarcObject = Factory.createPredicate("object", objectStr, pddlType.getImage());
        diarcObjects.add(diarcObject);
      }
    }
    facts.addAll(diarcObjects);

    // :goal parse pddl goal
    goal = generateDiarcPredicate(problem.getGoal());

    // TODO: how to handle pddl requirements?
  }

  private Predicate generateDiarcPredicate(Exp expression) {
    edu.tufts.hrilab.fol.Symbol diarcSymbol = generateDiarcPredicateHelper(expression);
    if (diarcSymbol.isPredicate()) {
      return (Predicate) diarcSymbol;
    } else {
      log.error("Error parsing PDDL expression: " + expression);
      return Factory.createPredicate("invalid", diarcSymbol);
    }
  }

  private edu.tufts.hrilab.fol.Symbol generateDiarcPredicateHelper(Exp expression) {
    if (expression.getValue() != null) {
      return Factory.createSymbol(expression.getValue().toString());
    } else if (expression.getAtom() != null && !expression.getAtom().isEmpty()) {
      // e.g., (holding self axe) --> holding(self,axe)
      List<Symbol> atom = expression.getAtom();
      List<String> diarcArgs = new ArrayList<>();
      atom.forEach(a -> diarcArgs.add(a.getImage()));
      String name = diarcArgs.remove(0);
      if (Operators.PDDL_TO_DIARC.containsKey(name)) {
        name = Operators.PDDL_TO_DIARC.get(name);
      }
      return Factory.createPredicate(name, diarcArgs.toArray(new String[diarcArgs.size()]));
    } else if (expression.getConnective().getImage().isEmpty() && expression.getChildren() != null && expression.getChildren().size() == 1) {
      return generateDiarcPredicateHelper(expression.getChildren().get(0));
    } else {
      // e.g., (= (inventory self log) 0) --> equals(inventory(self, log), 0)
      String name = expression.getConnective().getImage();
      if (Operators.PDDL_TO_DIARC.containsKey(name)) {
        name = Operators.PDDL_TO_DIARC.get(name);
      }
      List<edu.tufts.hrilab.fol.Symbol> diarcArgs = new ArrayList<>();
      for (Exp child : expression.getChildren()) {
        diarcArgs.add(generateDiarcPredicateHelper(child));
      }
      return Factory.createPredicate(name, diarcArgs.toArray(new edu.tufts.hrilab.fol.Symbol[diarcArgs.size()]));
    }
  }

  private void parseCondition(Exp pddlCond, ActionDBEntry.Builder actionBuilder) {
    if (pddlCond.getConnective().equals(Connective.OR)) {
      Condition.Disjunction disjunction = new Condition.Disjunction(ConditionType.PRE);
      for (Exp exp : pddlCond.getChildren()) {
        disjunction.or(generateDiarcPredicate(exp));
      }
      actionBuilder.addCondition(new Condition(disjunction));
    } else if (pddlCond.getConnective().equals(Connective.AND)) {
      // add each "AND" condition as a separate condition -- conjunction is implicit
      // this handles nested "and"s, which should be unnecessary in pddl specifications, but it does happen
      for (Exp exp : pddlCond.getChildren()) {
        parseCondition(exp, actionBuilder);
      }
    } else {
      actionBuilder.addCondition(new Condition(generateDiarcPredicate(pddlCond), ConditionType.PRE));
    }
  }

  private void parseEffect(Exp pddlEffect, ActionDBEntry.Builder actionBuilder) {
    if (pddlEffect.getConnective().equals(Connective.OR)) {
      log.error("PDDL action effects can not be disjunctive.");
      return;
    } else if (pddlEffect.getConnective().equals(Connective.AND)) {
      // add each "AND" effect as a separate effect -- conjunction is implicit
      // this handles nested "and"s, which should be unnecessary in pddl specifications, but it does happen
      for (Exp exp : pddlEffect.getChildren()) {
        parseEffect(exp, actionBuilder);
      }
    } else {
      actionBuilder.addEffect(new Effect(generateDiarcPredicate(pddlEffect), EffectType.SUCCESS));
    }
  }

  public static void main(String[] args) {
    if (args.length == 4 && args[0].equals("-o") && args[2].equals("-f")) {
    } else {
      System.out.println("Must pass in domain and problem files: -o <domainfile> -f <problemfile>");
      System.exit(-1);
    }

    String domainFile = args[1];
    String problemFile = args[3];

    PddlParser parser = new PddlParser();
    parser.parse(domainFile, problemFile);
  }
}
