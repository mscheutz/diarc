/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.cognex;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.ConsultantInterface;

import java.util.*;

public class CognexConsultant extends Consultant<CognexReference> implements ConsultantInterface {


  //TODO:brad: combine this with jobIndices
  protected Set<CognexJob> availableJobs;
  protected Map<String, CognexJob> boundJobs;
  //todo: the pose shouldn't be stored as a String.
  protected Map<CognexJob, String> graspPoses;

  protected Set<Symbol> lessSalientRefIds = new HashSet<>();

  protected static int refNumber = 0;

  public CognexConsultant() {
    super(CognexReference.class, "physobj");
    //Jobs
    availableJobs = new HashSet<>();
    CognexJob tray = new CognexJob("tray", "detector");
    availableJobs.add(tray);
    CognexJob milk = new CognexJob("detMilk", "detector");
    availableJobs.add(milk);

    boundJobs = new HashMap<>();
    boundJobs.put("tray", tray);

    graspPoses = new HashMap<>();

    //todo: remove. this should be taught
    graspPoses.put(milk,
            "[[15.01,-6.92,-356.49],[0.0209832,0.999589,-0.0176748,0.00825884],[0,-1,-1,4],[153.745,9E+09,9E+09,9E+09,9E+09,9E+09]]");
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return convertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    return type.cast(getReference(refId).result);
    //todo: add constraint filtering
  }

  @Override
  public boolean assertProperties(Map<Variable, Symbol> bindings, Double prob, List<Term> properties) {
    for (Symbol refId : bindings.values()) {
      CognexReference ref = getReference(refId);
      if (ref == null) {
        log.warn("[assertProperties] trying to assert properties to a null reference");
        return false;
      }
      ref.properties.addAll(properties);
      for (Term p : ref.properties) {
        if (boundJobs.containsKey(p.getName())) {
          ref.setCognexJob(boundJobs.get(p.getName()));
          //TODO:brad: this is a shortcut for now. We would probably want some sort of function that takes in all of the properties and returns the job name. That way we could support classifiers that use Job and Index as descriptors.
          break;
        }
      }
      for (CognexReference existingRef : getAllReferences()) {
        if (existingRef.refId != refId) {
          boolean matching = false;
          for (Term property : ref.properties) {
            for (Term eProperty : existingRef.properties) {
              if (property.getName().equals(eProperty.getName())) {
                matching = true;
                break;
              }
            }
            if (matching) {
              break;
            }
          }
          if (matching) {
            lessSalientRefIds.add(existingRef.refId);
          }
        }
      }
    }
    return true;
  }

  @Override
  public boolean assertProperties(Symbol refId, List<Term> properties) {
    CognexReference ref = getReference(refId);
    ref.properties.addAll(properties);
    for (Term p : ref.properties) {
      if (boundJobs.containsKey(p.getName())) {
        ref.setCognexJob(boundJobs.get(p.getName()));
        //TODO:brad: this is a shortcut for now. We would probably want some sort of function that takes in all of the properties and returns the job name. That way we could support classifiers that use Job and Index as descriptors.
        break;
      }
    }
    for (CognexReference existingRef : getAllReferences()) {
      if (existingRef.refId != refId) {
        boolean matching = false;
        for (Term property : ref.properties) {
          for (Term eProperty : existingRef.properties) {
            if (property.getName().equals(eProperty.getName())) {
              matching = true;
              break;
            }
          }
          if (matching) {
            break;
          }
        }
        if (matching) {
          lessSalientRefIds.add(existingRef.refId);
        }
      }
    }
    return true;
  }

  /**
   * Adds a binding between the human descriptor and the name of the job on the cognex, these doesn't necessarily need to be the same. The system should know about all of the job names, but it doesn't have to know about the human producable descriptors?
   * <p>
   * Once the binding is done it adds the proerty to the consultant. TODO:brad is there a way we could just get the jobs from the Cognex at start up? thT would make this a lot easier.
   *
   * @param descriptor human provided descriptor
   * @param jobName    name of job on the Cognex that is used to detect the thing associated with the provided descriptor
   */
  @TRADEService
  @Action
  public Symbol addDetectionType(Symbol descriptor, Symbol jobName) {
    //add pose name to properties consultant can handle

    String descriptorName = descriptor.getName();
    log.debug("[addDetectionType] " + descriptorName);

    List<Term> props = new ArrayList<>();
    props.add(Factory.createPredicate(descriptorName, "X:" + getKBName()));

    addPropertiesHandled(props);
    CognexJob job = null;
    for (CognexJob j : availableJobs) {
      if (j.getName().equals(jobName.getName())) {
        job = j;
        break;
      }
    }
    if (job == null) {
      //TODO:brad: handle this logging better
      log.debug("[addDetectionType] no job found for id " + jobName + " adding job to the set of known available jobs");
      //todo: remove this
      CognexJob newJob = new CognexJob(jobName.getName(), "detector");
      availableJobs.add(newJob);
      job = newJob;
    }
    boundJobs.put(descriptorName, job);
    return Factory.createSymbol(descriptorName);
  }


  public CognexReference createCognexRef(CognexJob job, List<Term> additionalProps) {

    //if there are any matching hypothetical bind them first
    for (CognexReference r : getAllReferences()) {
      if (r.cognexJob == job &&
              r.result == null &&
              //if there is a matching property for each additionalProp we're good
              additionalProps.stream().allMatch(p -> r.properties.stream().anyMatch(rp -> rp.getName().equals(p.getName())))
      ) {
        return r;
      }
    }

    //Get a new refId from consultant
    List<Variable> vars = new ArrayList<>();
    Variable var = Factory.createVariable("X");
    vars.add(var);
    Map<Variable, Symbol> refIds = createReferences(vars);

    List<Term> props = new ArrayList<>();
    String propertyName = job.getName();
    for (Map.Entry<String, CognexJob> e : boundJobs.entrySet()) {
      if (e.getValue().getName().equals(job.getName())) {
        propertyName = e.getKey();
      }
    }
    props.add(Factory.createPredicate(propertyName, "X"));
//        if (job.getType().equals("classifier")) {
//            props.add(Factory.createPredicate(index.toString(), "X"));
//        }

    //Notify consultant we have an instance of new object type
    assertProperties(refIds.get(var), props);
    CognexReference ref = getReference(refIds.get(var));

    //add location name to created reference
    return ref;
  }


  /**
   * @param descriptor
   * @return
   */
  public CognexJob getJobForDescriptor(String descriptor) {
    return boundJobs.get(descriptor);
  }


  //todo: should this actually be an action? additional sanity checks? should we route this through somewhere else? RR?
  @TRADEService
  @Action
  public CognexReference removeReference(Symbol refId) {
    return super.removeReference(refId);
  }

  //todo: does not handle general race conditions on ref management across consultants.
  //todo: duplicates code in the diarc PoseConsultant. We didn't want to implement a non-general
  //  version of reference sharing in the base Consultant class, but this is needed since
  //  the reference counter is no longer static in the base class.

  /**
   * Generates the new refId based on the refNumber counter, and increments. Informs all other
   * consultants with the same kbName of the current ref being allocated to attempt to maintain sync
   * and avoid duplicating refs.
   *
   * @return
   */
  @Override
  protected Symbol getNextReferenceId() {
    Symbol newReferenceId = Factory.createSymbol(kbName + "_" + refNumber++ + ":" + kbName);
    return newReferenceId;
  }


  @TRADEService
  @Action
  public CognexJob getCognexJobForDescriptor(Symbol descriptor) {
    return getJobForDescriptor(descriptor.getName());
  }

  @TRADEService
  @Action
  public CognexReference createCogRefWithProps(CognexJob j, List<Term> additionalProperties) {
    CognexReference ref = createCognexRef(j, additionalProperties);
    if (!additionalProperties.isEmpty()) {
      //TODO:brad:what happens if these are duplicate?
      assertProperties(ref.refId, additionalProperties);
    }
    return ref;
  }

  @TRADEService
  @Action
  public void bindCognexResult(CognexReference ref, CognexResult result, int indexIntoCognexResult) {
    log.debug("Binding: " + ref.refId + " to cognex result index: " + indexIntoCognexResult);
    ref.setResult(result);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("updateFOC")).call(void.class, ref.refId);
    } catch (TRADEException e) {
      log.error("[bindCognexResult] Exception calling updateFOC: ", e);
    }
  }

  @TRADEService
  @Action
  public List<Term> getEmptyProps() {
    return new ArrayList<>();
  }


  // set of actions which otherwise might be on a component above the consultant. unclear how this is organized for real robots

  /**
   * Returns the CognexJob able to detect the given reference, if it exists.
   *
   * @param ref The CognexReference to fetch the matching CognexJob of.
   * @return The CognexJob associated with this reference.
   */
  @TRADEService
  @Action
  public CognexJob getCognexJobForCognexReference(CognexReference ref) {
    return ref.cognexJob;
  }

  @TRADEService
  @Action
  public CognexReference getCognexReferenceForID(Symbol refId) {
    Symbol s = refId;
    CognexReference ret = getReference(s);
    if (ret == null) {
      log.warn("[getCognexReferenceForID] null reference found for id: " + s);
    }
    return ret;
  }

  //todo: this should have some data structure other than a string.

  public void setGraspPointForJob(CognexJob job, String graspPoint) {
    graspPoses.put(job, graspPoint);
  }

  //todo: string isn't really good enough.
  public String getGraspPoseForJob(CognexJob job) {
    return graspPoses.get(job);
  }

  @TRADEService
  @Action
  public CognexResult getMatchingResult(CognexReference toReBind, List<CognexResult> results) {
    return results.get(0);
  }

  //todo: naming? this is basically duplicating other functionality, but it's nice to have it all wrapped up in a single TRADE service.
  @TRADEService
  @Action
  public Symbol createCogRefWithProperty(Symbol jobDescriptor, Term property) {
    CognexReference ref = createCogRefWithProps(getCognexJobForDescriptor(jobDescriptor), new ArrayList<>(Arrays.asList(property)));
    //todo: shouldn't have to do this lookup this way. should have the refId already.
    for (CognexReference existingRef : getAllReferences()) {
      if (existingRef.equals(ref)) {
        return existingRef.refId;
      }
    }
    return null;
  }


//    /**
//     * runs a job for the given ?descriptor and saves the results in the cognex consultant
//     * @param descriptor human understandable binding to cognex job
//     */
//    @TRADEService
//    @Action
//    public void observeDescriptor(Symbol descriptor){
//        CognexJob job=getCognexJobForDescriptor(descriptor);
//        List<CognexResult> cameraResults= getCameraData(job.getName());
//        bindResultsRecursive(job,cameraResults,0);
//    }

  @Override
  public void addReference(CognexReference newRef) {
    super.addReference(newRef);
    for (CognexReference ref : getAllReferences()) {
      if (ref.properties.containsAll(newRef.properties) && newRef.properties.containsAll(ref.properties)) {
        lessSalientRefIds.add(ref.refId);
      }
    }
  }

  @Override
  public Map<Symbol, Double> getActivatedEntities() {
    log.debug("[getActivatedEntities]");
    Map<Symbol, Double> activatedEntities = super.getActivatedEntities();
    Map<Symbol, Double> toReturn = new HashMap<>();
    for (Symbol refId : activatedEntities.keySet()) {
      if (!lessSalientRefIds.contains(refId)) {
        toReturn.put(refId, activatedEntities.get(refId));
      }
    }
    return toReturn;
  }
}