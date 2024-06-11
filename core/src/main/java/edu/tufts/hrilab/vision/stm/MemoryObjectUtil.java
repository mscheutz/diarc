/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.stm;

import alice.tuprolog.MalformedGoalException;
import alice.tuprolog.NoMoreSolutionException;
import alice.tuprolog.NoSolutionException;
import alice.tuprolog.Prolog;
import alice.tuprolog.SolveInfo;
import alice.tuprolog.Struct;
import alice.tuprolog.Var;
import edu.tufts.hrilab.belief.provers.prolog.PrologUtil;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 *
 * @author Evan Krause evan.krause@tufts.edu
 */
public class MemoryObjectUtil {

  private static Logger log = LoggerFactory.getLogger(MemoryObjectUtil.class);

  /**
   * Helper method to get the angle (in radians) between the grasp orientation
   * and the passed in vector..
   *
   * @param grasp
   * @return
   */
  static public double getAngleFromAngle(Grasp grasp, Vector3d angleVector) {
    Vector3d xVec = new Vector3d(1, 0, 0);
    Quat4d graspOrient = grasp.getOrientation();
    Vector3d graspDirVec = new Vector3d();
    Matrix4d rotMatrix = new Matrix4d(graspOrient, new Vector3d(0, 0, 0), 1.0);
    rotMatrix.transform(xVec, graspDirVec);
    double angle = angleVector.angle(graspDirVec);
    return Math.abs(angle);
  }

  /**
   * Get the highest confidence value for each term in list along with the
   * bindings (variable to memory object identifier (e.g., object26)) for the
   * returned confidence values.
   *
   * @param rootMO root of the scene graph to be searched
   * @param constraints to be satisfied
   * @param bindings return bindings for highest confidence match
   * @return list of confidence values, one for each constraint Term
   */
  static public List<Float> getConfidence(MemoryObject rootMO, List<? extends Term> constraints, Map<Variable, Symbol> bindings) {
    List<Float> bestConfs = new ArrayList<>();
    Map<Variable, MemoryObject> bestBindings = new HashMap<>();
    float bestConf = 0;
    List<Map<Variable, MemoryObject>> moBindings = new ArrayList<>();
    if (getMemoryObjectBindings(rootMO, constraints, moBindings)) {
      // find confidence values for each bindings instance
      for (Map<Variable, MemoryObject> moBindingsInstance : moBindings) {
        List<Float> tmpConfs = new ArrayList<>();
        float tmpConf = getConfidence(rootMO, constraints, moBindingsInstance, tmpConfs);
        if (tmpConf > bestConf) {
          bestConfs = tmpConfs;
          bestBindings = moBindingsInstance;
          bestConf = tmpConf;
        }
      }

      // fill return bindings with bestBindings
      bindings.putAll(convertToIdentifierBindings(bestBindings));
    } else {
      log.warn("[getConfidence] no bindings found for constraints: " + constraints);
    }
    return bestConfs;
  }

  /**
   * Helper method to calculate the confidence values for the current bindings.
   *
   * @param constraints
   * @param bindingsInstance
   * @param confs
   * @return
   */
  static private float getConfidence(MemoryObject rootMO, List<? extends Term> constraints, Map<Variable, MemoryObject> bindingsInstance, List<Float> confs) {
    for (Term constraint : constraints) {
      if (constraint.size() == 1) {
        // mo property
        Symbol arg = constraint.get(0);
        if (arg.isVariable()) {
          // TODO: this isn't sufficient -- need to get confidence for single term only, not all mo's terms
          Variable arg_v = (Variable) arg;
          confs.add((float) bindingsInstance.get(new Variable(arg_v.getName(), arg_v.getType())).getDetectionConfidence());
        } else {
          // pre-bound constraint (i.e., no Variable) -- just try to look up in scene graph
          confs.add((float) rootMO.getMemoryObject(arg).getDetectionConfidence());
        }

      } else if (constraint.size() == 2) {
        // relation
        // find MOs
        List<MemoryObject> mos = new ArrayList<>();
        for (int i = 0; i < constraint.size(); ++i) {
          Symbol arg = constraint.get(i);
          if (arg.isVariable()) {
            // find MO in bindings results
            mos.add(bindingsInstance.get((Variable) arg));
          } else {
            // pre-bound constraint (i.e., no Variable) -- just try to find MO in scene graph
            mos.add(rootMO.getMemoryObject(arg));
          }
        }
        // find relation
        for (MemoryObjectRelation relation : mos.get(0).getRelations(constraint)) {
          if (relation.getRelatedObject().equals(mos.get(1))) {
            confs.add(relation.getConfidence());
            break;
          }
        }

      } else {
        log.error("[getConfidence] can't handle constraints with " + constraint.size() + " args.");
        confs.add(0.0f);
      }
    }

    float returnConf = 0.0f;

    if (confs.size()
            == constraints.size()) {
      returnConf = 1.0f;
      for (float conf : confs) {
        returnConf *= conf;
      }
    } else {
      log.error("[getConfidence] didn't find confidence value for each constraint.");
    }
    return returnConf;
  }

  /**
   * Helper method to convert bindings to MemoryObjects to bindings of
   * MemoryObject identifiers.
   *
   * @param bindings
   * @return
   */
  static private Map<Variable, Symbol> convertToIdentifierBindings(Map<Variable, MemoryObject> bindings) {
    Map<Variable, Symbol> identifierBindings = new HashMap<>();
    for (Variable var : bindings.keySet()) {
      Symbol identifier = bindings.get(var).getIdentifier();
      identifierBindings.put(var, identifier);
    }
    return identifierBindings;
  }

  /**
   * Get list of MemoryObject bindings from the scene graph that meet the
   * constraints.
   *
   * @param mo
   * @param constraints
   * @param bindings
   * @return if bindings were found for all free variables
   */
  static public boolean getMemoryObjectBindings(MemoryObject mo, List<? extends Term> constraints, List<Map<Variable, MemoryObject>> bindings) {
    List<Map<Variable, Symbol>> identifierBindings = new ArrayList<>();
    if (getMemoryObjectIdentifierBindings(mo, constraints, identifierBindings)) {
      for (Map<Variable, Symbol> identifierBindingsInstance : identifierBindings) {
        Map<Variable, MemoryObject> bindingsInstance = new HashMap<>();
        for (Variable var : identifierBindingsInstance.keySet()) {
          Symbol identifier = identifierBindingsInstance.get(var);
          MemoryObject identifierMO = mo.getMemoryObject(identifier);
          bindingsInstance.put(var, identifierMO);
        }
        bindings.add(bindingsInstance);
      }
      return true;
    } else {
      return false;
    }
  }

  /**
   * Get list of MemoryObject identifier bindings (e.g., object31) from the
   * scene graph that meet the constraints.
   *
   * @param mo
   * @param constraints
   * @param bindings
   * @return if bindings were found for all free variables
   */
  static public boolean getMemoryObjectIdentifierBindings(MemoryObject mo, List<? extends Term> constraints, List<Map<Variable, Symbol>> bindings) {
    // populate prolog with scene graph info
    Prolog prolog = new Prolog();
    populateProlog(prolog, mo);

    // query prolog for constraints bindings
    StringBuilder queryBuilder = new StringBuilder();
    for (int i = 0; i < constraints.size(); ++i) {
      queryBuilder.append(PrologUtil.makePrologSafe(constraints.get(i)));
      if (i < constraints.size() - 1) {
        queryBuilder.append(","); //conjunction in prolog
      }
    }
    queryBuilder.append(".");

    try {
      SolveInfo sln = prolog.solve(queryBuilder.toString());
      if (!sln.isSuccess()) {
        return false;
      }
      while (sln.isSuccess()) {
        Map<Variable, Symbol> bindingsInstance = new HashMap<>();
        Symbol varBinding;
        List<Var> prologBindings = sln.getBindingVars();
        for (Var v : prologBindings) {
          if (v.isBound()) {
            if (v.getTerm().isAtom()) {
              varBinding = new Symbol(((Struct) v.getTerm()).getName());
            } else if (v.getTerm() instanceof alice.tuprolog.Number) {
              varBinding = new Symbol(Double.toString(((alice.tuprolog.Number) v.getTerm()).doubleValue()));
            } else if (v.getTerm().isCompound()) {
              Struct s = (Struct) v.getTerm();
              varBinding = PrologUtil.convertToAdeForm(s);
            } else {
              log.error(String.format("No conversion rule applicable to %s.", v.getTerm()));
              continue;
            }
            // TODO: get "object" from some global place instead of hardcoding
            bindingsInstance.put(new Variable(v.getName(), PredicateHelper.varType), varBinding);
          }
        }

        bindings.add(bindingsInstance);

        if (prolog.hasOpenAlternatives()) {
          sln = prolog.solveNext();
        } else {
          break;
        }
      }
    } catch (MalformedGoalException mge) {
      log.error(String.format("Prolog Query Error during query '%s': ", queryBuilder.toString()), mge);
      bindings.clear();
    } catch (NoMoreSolutionException nmse) {
      log.error(String.format("No more solutions found for query %s", queryBuilder.toString()), nmse);
    } catch (NoSolutionException nse) {
      log.error(String.format("No solution found for query %s", queryBuilder.toString()), nse);
    }

    return true;
  }

  /**
   * Convenience method to pass in a single constraint.
   *
   * @param mo
   * @param constraints
   * @param bindings
   * @return
   */
  static public boolean getMemoryObjectBindings(MemoryObject mo, Term constraints, List<Map<Variable, MemoryObject>> bindings) {
    List<Term> constraintsList = new ArrayList<>();
    constraintsList.add(constraints);
    return getMemoryObjectBindings(mo, constraintsList, bindings);
  }

  /**
   * Convenience method to pass in a single constraint.
   *
   * @param mo
   * @param constraints
   * @param bindings
   * @return if bindings were found for all free variables
   */
  static public boolean getMemoryObjectIdentifierBindings(MemoryObject mo, Term constraints, List<Map<Variable, Symbol>> bindings) {
    List<Term> constraintsList = new ArrayList<>();
    constraintsList.add(constraints);
    return getMemoryObjectIdentifierBindings(mo, constraintsList, bindings);
  }

  static public boolean applyDefinition(MemoryObject mo, Term name, List<Term> descriptors) {
    // TODO: add more checks on Term
    Variable var = (Variable) name.get(0);
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    if (!getMemoryObjectIdentifierBindings(mo, descriptors, bindings)) {
      return false;
    }
    for (Map<Variable, Symbol> binding : bindings) {
      Symbol id = binding.get(var);
      if (id == null) {
        log.error("[applyDefinition] no matching variable in binding list. var: " + var + " binding: " + binding);
        continue;
      }
      MemoryObject node = mo.getMemoryObject(id);

      // replace name with correct node Variable
      Term newName = new Predicate(name.getName(), node.getVariable());

      // finally, add definition label!
      //TODO: what should conf val be?
      node.addDescriptor(newName, 1.0f);
    }
    return true;
  }

  /**
   * Helper method to populate an existing prolog instance with a scene graph.
   *
   * @param prolog
   * @param mo
   */
  static private void populateProlog(Prolog prolog, MemoryObject mo) {
    Set<Term> sceneGraphDescription = getBoundSceneGraphDescriptors(mo);

    for (Term boundDesciptor : sceneGraphDescription) {
      String prologTerm = PrologUtil.makePrologSafe(boundDesciptor);

      // assert to prolog
      try {
        prolog.solve("assert(" + prologTerm + ").");
      } catch (MalformedGoalException e) {
        log.error("Belief Component Assertion Error: ", e);
      }
    }
  }

  /**
   * Helper method to get an entire bound scene graph description given the root of the scene graph.
   *
   * @param mo
   * @return
   */
  static public Set<Term> getBoundSceneGraphDescriptors(MemoryObject mo) {
    Set<Term> sceneGraphDescriptors = new HashSet<>();
    Queue<MemoryObject> frontier = new ArrayDeque<>();
    frontier.add(mo);
    while (!frontier.isEmpty()) {
      MemoryObject currMO = frontier.poll();
      frontier.addAll(currMO.getChildren());

      // add descriptors
      Symbol binding = currMO.getIdentifier();
      for (Term descriptor : currMO.getDescriptors()) {

        // build term -- bind variable to token id (e.g., "object4")
        List<Symbol> args = new ArrayList<>();
        for (Symbol arg : descriptor.getArgs()) {
          if (arg.isVariable()) {
            args.add(binding);
          } else {
            args.add(arg);
          }
        }
        sceneGraphDescriptors.add(new Term(descriptor.getName(), args));
      }

      // add relations
      Variable thisVar = currMO.getVariable();
      Symbol thisBinding = currMO.getIdentifier();
      for (MemoryObjectRelation relation : currMO.getRelations()) {

        // bind variable to token id (e.g., "object4")
        Variable otherVar = relation.getRelatedObject().getVariable();
        Symbol otherBinding = relation.getRelatedObject().getIdentifier();
        Term relationTerm = relation.getDescriptor();
        List<Symbol> args = new ArrayList<>();
        for (Symbol arg : relationTerm.getArgs()) {
          if (arg.isVariable()) {
            if (arg.equals(thisVar)) {
              args.add(thisBinding);
            } else if (arg.equals(otherVar)) {
              args.add(otherBinding);
            } else {
              log.error("[addRelation] relation does not contain correct variables. arg: " + arg
                      + " thisVar: " + thisVar
                      + " otherVar: " + otherVar);
            }
          } else {
            args.add(arg);
          }
        }
        sceneGraphDescriptors.add(new Term(relationTerm.getName(), args));
      }
    }

    return sceneGraphDescriptors;
  }

  /**
   * Helper method to get an entire scene graph description (i.e., unbound
   * descriptors) given the root of the scene graph.
   *
   * @param mo
   */
  static public Set<Term> getSceneGraphDescriptors(MemoryObject mo) {
    Set<Term> sceneGraphDescriptors = new HashSet<>();
    Queue<MemoryObject> frontier = new ArrayDeque<>();
    frontier.add(mo);
    while (!frontier.isEmpty()) {
      MemoryObject currMO = frontier.poll();
      frontier.addAll(currMO.getChildren());

      // add descriptors
      sceneGraphDescriptors.addAll(currMO.getDescriptors());

      // add relations
      for (MemoryObjectRelation relation : currMO.getRelations()) {
        sceneGraphDescriptors.add(relation.getDescriptor());
      }
    }

    return sceneGraphDescriptors;
  }

}
