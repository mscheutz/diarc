/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigEffectMeasures;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.gson.stream.JsonWriter;
import com.google.gson.Gson;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigPerformanceMeasures;

import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Tree to keep track of the probability an action will succeed and the expect time given arguments
 * Using tree instead of table for easier lookup of queries using unbound args
 */
public class PerformanceMeasuresNode {
  // create probability interface which has total, numSuccess, uncertainty
  private Symbol value;
  private List<Symbol> priorArgs; // other relevant bound arguments to action
  private EffectProbabilities effectProbabilities;
  private int totalExperience; // used for both holds and time
  private int success; 
  private double successTimeMean; // TODO: change this to a probability class
  private double successTimeVar;
  private double successSumSqDiff;
  private double failureTimeMean; // TODO: change this to a probability class
  private double failureTimeVar;
  private double failureSumSqDiff;
  // map value of argument to a child probability node
  private Map<Symbol, PerformanceMeasuresNode> childNodes;
  private final static Logger log = LoggerFactory.getLogger(PerformanceMeasuresNode.class);

  public PerformanceMeasuresNode(Symbol value) {
    this.value = value;
    totalExperience = 0;
    successSumSqDiff = 0;
    success = 0;
    successTimeMean = 0.0;
    successTimeVar = 0;
    failureTimeMean = 0.0;
    failureTimeVar = 0;
    failureSumSqDiff = 0;
    childNodes = new HashMap<>();
    effectProbabilities = new EffectProbabilities();
  }

  public void setEffectInformation(List<Predicate> effects) {
    effectProbabilities.setEffectInformation(effects);
  }

  /**
   * recursively save performance measures using the json writer
   * @param writer json writer
   * @param gson used to convert config classes to json
   */
  public boolean savePerformanceMeasures(JsonWriter writer, Gson gson) {
    try {
      if (!value.getName().equalsIgnoreCase("root")) {
        writer.name(value.toString()); // set name
        writer.beginObject();
      }
      if (!childNodes.isEmpty()) {
        for (PerformanceMeasuresNode performanceMeasuresNode : childNodes.values()) {
          performanceMeasuresNode.savePerformanceMeasures(writer, gson);
        }
      } else {
        writer.name("performanceMeasures");
        ConfigPerformanceMeasures performanceMeasures = generateMeasuresGsonObject();
        gson.toJson(performanceMeasures, ConfigPerformanceMeasures.class, writer);
      }
      if (!value.getName().equalsIgnoreCase("root")) {
        writer.endObject();
      }
    } catch (IOException ex) {
      return false;
    }
    return true;
  }

  /**
   * generate gson object to save models
   */
  private ConfigPerformanceMeasures generateMeasuresGsonObject() {
    ConfigPerformanceMeasures config = new ConfigPerformanceMeasures();
    config.setExperience(totalExperience);
    config.setSuccess(success);
    config.setSuccessTimeMean(successTimeMean);
    config.setSuccessTimeVar(successTimeVar);
    config.setEffectMeasures(effectProbabilities.createEffectMeasuresGson());
    config.setSuccessSumSqDiff(successSumSqDiff);
    config.setFailureTimeMean(failureTimeMean);
    config.setFailureTimeVar(failureTimeVar);
    config.setFailureSumSqDiff(failureSumSqDiff);
    return config;
  }

  /**
   * create a new node and add to map of children using value as key
   * @param value value to associate new node to in childNodes
   */
  private void createNewNode(Symbol value) {
    PerformanceMeasuresNode node = new PerformanceMeasuresNode(value);
    childNodes.put(value, node);
  }

  private void createNewNodeEffects(Symbol value) {
    PerformanceMeasuresNode node = new PerformanceMeasuresNode(value);
    node.setEffectInformation(effectProbabilities.getEffects());
    childNodes.put(value, node);
  }

//////////////////////////////////////////////////
//  Update Models
//////////////////////////////////////////////////
  /**
   * incremental update of time distribution after each execution
   * @param time time to complete the action
   * @param holds did the effect hold / did the action succeed
   */
  private void updateTimePriors(double time, boolean holds) {
    if (holds) {
      double deltaMean = time - successTimeMean;
      double newMean = successTimeMean + (deltaMean / (double) success);
      double deltaNewMean = time - newMean;
        /*
          Welford's online algorithm
          sumSqDiff = sumSqDiff + (obs - prior) * (obs - newMean)
          var = sumSqDiff / (n)
         */
      successTimeMean = newMean;
      successSumSqDiff = successSumSqDiff + deltaMean * deltaNewMean;
      successTimeVar = successSumSqDiff / (double) success;
    } else {
      double failures = totalExperience - success;
      log.debug("[updating failure time] total: " + totalExperience + " success " + success + " failures " + failures);
      double deltaMean = time - failureTimeMean;
      double newMean = failureTimeMean + (deltaMean / failures);
      double deltaNewMean = time - newMean;
      failureTimeMean = newMean;
      failureSumSqDiff = failureSumSqDiff + deltaMean * deltaNewMean;
      failureTimeVar = failureSumSqDiff / failures;
    }
  }

  /**
   * batch update of time distribution (reading performance measures from file)
   * @param successMean mean of time distribution to update current mean
   * @param successVar var of time distribution to update current timeVar
   * @param experience how many times has the action been completed, used for weighted sum
   */
  private void updateTimePriors(double successMean, double successVar, double failureMean, double failureVar, double experience, double holds) {
    double tSuccess = success + holds;
    double ws1 = success / tSuccess;
    double ws2 = holds / tSuccess;

    // avg = w1 * prior + w2 * new
    successTimeMean = ws1 * successTimeMean + ws2 * successMean;
    // var = w1^2 * prior + w2^2 * new
    successTimeVar = Math.pow(ws1, 2) * successTimeVar + Math.pow(ws2, 2) * successVar;

    double tFailure = success + holds;
    double wf1 = success / tFailure;
    double wf2 = holds / tFailure;

    // avg = w1 * prior + w2 * new
    failureTimeMean = wf1 * failureTimeMean + wf2 * failureMean;
    // var = w1^2 * prior + w2^2 * new
    failureTimeVar = Math.pow(wf1, 2) * failureTimeVar + Math.pow(wf2, 2) * failureVar;
  }

  /**
   * incremental update of experience and time distribution
   * @param argValues input values to action parameters, used to find correct performance node
   * @param isSuccess did the effect hold / did the action succeed
   * @param time time to complete the action
   */
  public void updatePerformanceModels(List<Symbol> argValues, boolean isSuccess, double time) {
    totalExperience++;
    boolean shouldUpdateTime = true;
    if (isSuccess) {
      success ++;
      if (success == 1) {
        shouldUpdateTime = false;
        successTimeMean = time;
      }
    } else {
      if ((totalExperience - success) == 1) { // totalExperience is already updated, so if there weren't any failures before there should be a difference of 1
        shouldUpdateTime = false;
        failureTimeMean = time;
      }
    }
    if (shouldUpdateTime) {
      updateTimePriors(time, isSuccess);
    }

    //if (totalExperience > 1 ) {
    //  updateTimePriors(time, isSuccess);
    //} else {
    //  if (isSuccess) {
    //    successTimeMean = time;
    //  } else {
    //    failureTimeMean = time;
    //  }
    //}
    // update child nodes
    if (!argValues.isEmpty()) {
      Symbol val = argValues.get(0);
      List<Symbol> restArg = argValues.subList(1, argValues.size());
      // if no specific value used, then update all childnodes
      if (val == null) {
        // TODO: tmf: how can there be an instance where one of the arguments isn't bound after executiion
        //            probably shouldn't ever happen
        for (PerformanceMeasuresNode node : childNodes.values()) {
          node.updatePerformanceModels(restArg, isSuccess, time);
        }
        log.warn("No argument specified 'null' updating all child performance models. This probably shouldn't happen");
        return;
      }
      // create new probabilityNode if none exists for value
      if (!childNodes.containsKey(val)) {
        createNewNodeEffects(val);
      }
      // update child node
      PerformanceMeasuresNode node = childNodes.get(val);
      node.updatePerformanceModels(restArg, isSuccess, time);
    } else {
      log.debug("exp " + totalExperience + " isSuccess " + success + " successTimeMean " + successTimeMean + " successTimeVar " + successTimeVar + " failTimeMean " + failureTimeMean + " failTimeVar " + failureTimeVar);
    }
  }

  /**
   * incremental update of effect models
   * @param effectHolds map from effect predicate to boolean value
   * @param argValues input vaules to aciton parameters, used to update correct performance node
   */
  public void updateEffectPerformanceModels(Map<Predicate, Boolean> effectHolds, List<Symbol> argValues) {
    if (effectHolds.isEmpty()) {
      for (Predicate effect : effectProbabilities.getEffects()) {
        effectHolds.put(effect, false);
      }
    }
    effectProbabilities.updateModels(effectHolds);
    if (!argValues.isEmpty()) {
      Symbol val = argValues.get(0);
      List<Symbol> restArg = argValues.subList(1, argValues.size());
      if (!childNodes.containsKey(val)) {
        createNewNodeEffects(val);
      }
      PerformanceMeasuresNode node = childNodes.get(val);
      node.updateEffectPerformanceModels(effectHolds, restArg);
    }
  }

  /**
   * batch update of time distribution (reading performance measures from file)
   * @param argValues input values to action parameters, used to find correct performance node
   * @param experience total number of times action has been executed
   * @param holds number of times the effect hold / did the action succeed
   * @param successTimeMean mean time to complete the action with specified args
   * @param successTimeVar variance time to complete the action with specified args
   */
  public void updatePerformanceModels(List<Symbol> argValues, double experience, double holds,
                                      double successTimeMean, double successTimeVar, double successSumSqDiff,
                                      double failureTimeMean, double failureTimeVar, double failureSumSqDiff,
                                      ConfigEffectMeasures effMeasures, Map<String, List<Predicate>> dependentGroups) {
    // successTimeMean = holdTimeMean * priorExperience / (priorExperience + experience) + successTimeMean * (experience) / (totalExperience + experience)
    if (totalExperience == 0) {
      this.successTimeMean = successTimeMean;
      this.successTimeVar = successTimeVar;
      this.successSumSqDiff = successSumSqDiff;
      this.failureTimeMean = failureTimeMean;
      this.failureTimeVar = failureTimeVar;
      this.failureSumSqDiff = failureSumSqDiff;
    } else {
      updateTimePriors(successTimeMean, successTimeVar, failureTimeMean, failureTimeVar, experience, holds); // FIXME: need to save sumSqDiff and variance, then load values
    }
    success += holds;
    totalExperience += experience;
    effectProbabilities.updateModels(effMeasures.getMarginalMeasures(), effMeasures.getJointDistributions(), dependentGroups, (int)experience);


    if (!argValues.isEmpty()) {
      Symbol val = argValues.get(0);
      List<Symbol> restArg = argValues.subList(1, argValues.size());//TODO: check null pointer
      // if no specific value used, then update all childnodes -- seems unlikely, but probably should update?
      if (val == null) {
        for (PerformanceMeasuresNode node : childNodes.values()) {
          node.updatePerformanceModels(restArg, experience, holds, successTimeMean, successTimeVar, successSumSqDiff, failureTimeMean, failureTimeVar, failureSumSqDiff, effMeasures, dependentGroups);
        }
        return;
      }
      if (!childNodes.containsKey(val)) {
        createNewNode(val);
      }
      PerformanceMeasuresNode node = childNodes.get(val);
      node.updatePerformanceModels(restArg, experience, holds, successTimeMean, successTimeVar, successSumSqDiff, failureTimeMean, failureTimeVar, failureSumSqDiff, effMeasures, dependentGroups);
    }
  }

//////////////////////////////////////////////////
// Get Models
//////////////////////////////////////////////////
  public PerformanceMeasuresNode getPerformanceMeasures(List<Symbol> argValues) {
    // if there is a matching child node with a value not equal to null, then return that child node
    // otherwise return this
    if (!argValues.isEmpty()) {
      Symbol val = argValues.get(0);
      List<Symbol> rest = argValues.subList(1, argValues.size()); // should return empty list if argValues size is 1
      // assuming null represents not bound
      if (val != null) {
        PerformanceMeasuresNode node = childNodes.get(val);
        if (node != null) {
          return node.getPerformanceMeasures(rest);
        } else {
          // if the value is null, then use heuristics to generate a new performance model combining relevant children
          // for now ignore and return this performance measures node with values reflecting sum of children
          /* TODO: use heuristic to determine how to handle
                   currently summing over all children
                   but could sum over related children
          */
        }
      }
    }
    return this;
  }

  public Map<Predicate, Boolean> sampleEffects(Map<Predicate, Boolean> given) {
    return effectProbabilities.sampleEffects(given);
  }

  //public boolean sampleEffect(Predicate predicate, Map<Predicate, Boolean> given) {
  //  return effectProbabilities.sampleEffect(predicate, given);
  //}

  public Pair<Integer, Integer> getExperience() {
    return Pair.of(success, totalExperience);
  }

  public Pair<Double, Double> getTimeDistribution(boolean holds) {
    return holds ? Pair.of(successTimeMean, successTimeVar) : Pair.of (failureTimeMean, failureTimeVar);
    //return Pair.of(successTimeMean, successTimeVar);
  }

  /**
   *
   * @param argValues
   * @return mean, variance
   */
  public Pair<Double, Double> getTimeProbability(List<Symbol> argValues, boolean holds) {
    Pair<Double, Double> output = null;
    if (argValues.isEmpty()) {
      if (totalExperience > 0) {
        output = Pair.of(successTimeMean, successTimeVar);
      }
    } else {
      // get the value
      Symbol val = argValues.get(0);
      List<Symbol> rest = argValues.subList(1, argValues.size());//TODO: check null pointer
      // if the value is null, then get the mean probability of all children
      if (val == null) {
        // TODO: fix the following to take into account variance
        // calculate distribution of the combined child distributions
        double time = 0.0;
        int totalChildExperience = 0;
        double variance = 0.0;
        // time = (w1 * t1 + w2 * t2 ... + wn * tn) / sum(wi)
        //    where wi = experience of ith child
        // var = (w1^2 * v1 + w2^2 * v2 ... + wn^2 * vn) / sum(wi)^2
        for (PerformanceMeasuresNode node : childNodes.values()) {
          Pair<Double,Double> timeDistribution = node.getTimeProbability(rest, holds);
          Pair<Integer, Integer> experience = node.getExperience(rest);
          time += timeDistribution.getLeft() * experience.getRight();
          variance += timeDistribution.getRight() * Math.pow(experience.getRight(), 2);
          totalChildExperience += experience.getRight();
        }
        output = Pair.of(time / totalChildExperience, variance / Math.pow(totalChildExperience, 2));
      } else {
        PerformanceMeasuresNode node = childNodes.get(val);
        if (node != null) {
          output = node.getTimeProbability(rest, holds);
        }
      }
    }
    if (output != null) {
      log.debug("probability effect will hold is: " + output.toString());
    }
    return output;
  }

  public Pair<Integer, Integer> getExperience(List<Symbol> argValues) {
    Pair<Integer, Integer> output = null;
    if (argValues.isEmpty()) {
      output = Pair.of(success, totalExperience);
    } else {
      // get the value
      Symbol val = argValues.get(0);
      List<Symbol> rest = argValues.subList(1, argValues.size()); // should return empty list if argValues size is 1
      // assuming null represents not bound
      // if the value is null, then get the mean probability of all children
      if (val == null) {
        int sumHold = 0;
        int sumTotal = 0;
        for (PerformanceMeasuresNode node : childNodes.values()) {
          Pair<Integer, Integer> childExperience = node.getExperience(rest);
          sumHold += childExperience.getLeft();
          sumTotal += childExperience.getRight();
        }
        output = Pair.of(sumHold, sumTotal);
      } else {
        PerformanceMeasuresNode node = childNodes.get(val);
        if (node != null) {
          output = node.getExperience(rest);
        } else {
          /* TODO: use heuristic to determine how to handle
                   currently summing over all children
                   but could sum over related children
          */
          int sumHold = 0;
          int sumTotal = 0;
          for (PerformanceMeasuresNode childNode : childNodes.values()) {
            Pair<Integer, Integer> childExperience = childNode.getExperience(rest);
            sumHold += childExperience.getLeft();
            sumTotal += childExperience.getRight();
          }
          output = Pair.of(sumHold, sumTotal);
        }
      }
    }
    log.debug("Effect experience is: " + output);
    return output;
  }
}
