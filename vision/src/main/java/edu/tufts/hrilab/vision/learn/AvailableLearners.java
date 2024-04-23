/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.learn;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.learn.swig.NativeLearner.LearnerType;
import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Evan Krause evan.krause@tufts.edu
 */


public class AvailableLearners {

  private HashMap<LearnerType, LearnerDetail> typesDetail = new HashMap<>();
  private static final Logger log = LoggerFactory.getLogger(AvailableLearners.class);

  public AvailableLearners(final int imgWidth, final int imgHeight, final String configFile) {
    //TODO: pass in config file
    typesDetail.put(LearnerType.DEFINITION, new LearnerDetail(LearnerType.DEFINITION, imgWidth, imgHeight, ImageProcessorType.DEFINITIONVALIDATOR));
    typesDetail.put(LearnerType.INSTANCE, new LearnerDetail(LearnerType.INSTANCE, imgWidth, imgHeight, ImageProcessorType.GLOBALFEATUREVALIDATOR));
  }

  public Term learn(Term learningTerm) {
    // separate the name and target description
    // definitionOf(description, name) e.g., definitionOf(partOf(orange,knife), handle)
    // instanceOf(description, name) e.g., instanceOf(object_4, knife)
    Term nameToLearn;
    List<Term> arg1 = PredicateHelper.convertToVisionForm(learningTerm.get(1));
    if (arg1.size() == 1) {
      nameToLearn = arg1.get(0);
    } else {
      log.error("[learn] couldn't learn: " + learningTerm);
      return null;
    }

    List<Term> targetObjectDescriptors = new ArrayList<>();
    if (PredicateHelper.isObjectRef(learningTerm.get(0))) {
      targetObjectDescriptors.add(new Term(learningTerm.get(0).getName(), nameToLearn.get(0)));
    } else {
      targetObjectDescriptors.addAll(PredicateHelper.convertToVisionForm(learningTerm.get(0)));
    }

    //TODO: move hardcoded strings to config file!!
    if (learningTerm.getName().equals("instanceOf")) {
      learnInstanceOf(learningTerm, nameToLearn, targetObjectDescriptors);
    } else if (learningTerm.getName().equals("definitionOf")) {
      learnDefinitionOf(learningTerm, nameToLearn, targetObjectDescriptors);
    } else {
      log.error("[learn] can't handle learning term: " + learningTerm);
      return null;
    }

    return nameToLearn;
  }

  public Term unlearn(Term learningTerm) {
    // find which argument is the name, and which one is the description since it
    // can be either way: definitionOf(handle, partOf(orange,knife)) OR definitionOf(partOf(orange,knife), handle)
    Term nameToUnlearn;
    List<Term> targetObjectDescriptors;
    List<Term> arg0 = PredicateHelper.convertToVisionForm(learningTerm.get(0));
    List<Term> arg1 = PredicateHelper.convertToVisionForm(learningTerm.get(1));

    // second arg is always the name of the thing being learned
    if (arg1.size() == 1) {
      nameToUnlearn = arg1.get(0);
      targetObjectDescriptors = arg0;
    } else {
      log.error("[learn] couldn't learn: " + learningTerm);
      return null;
    }

    //TODO: move hardcoded strings to config file!!
    if (learningTerm.getName().equals("definitionOf")) {
      unlearnDefinitionOf(learningTerm, nameToUnlearn, targetObjectDescriptors);
    } else {
      log.error("[learn] can't handle learning term: " + learningTerm);
      return null;
    }

    return nameToUnlearn;
  }

  private void learnInstanceOf(Term learningTerm, Term nameToLearn, List<Term> targetObjectDescriptors) {
    log.debug("[learnInstanceOf]: " + learningTerm);

    // get/instantiate search manager for object to learn
    SearchManager objectSM = null;
    if (PredicateHelper.isObjectRef(targetObjectDescriptors)) {
      // if thing to learn is a POWER ref (e.g., "instanceOf(objects_3, knife)")
      Symbol objectRef = new Symbol(targetObjectDescriptors.get(0).getName());
      objectSM = Vision.availableSearchTypes.getInstance(this, Vision.consultant.getTypeId(objectRef));
    } else {
      // else thing to learn is a description
      objectSM = Vision.availableSearchTypes.getInstance(this, targetObjectDescriptors, true);
    }

    if (objectSM == null) {
      log.error("[learn] could not get/instantiate search manager for: " + targetObjectDescriptors);
      return;
    }

    // instantiate learner processor
    LearnerDetail learnerDetail = typesDetail.get(LearnerType.INSTANCE);
    Learner learner = new Learner(learnerDetail);
    learner.addProcessingDescriptor(learningTerm, objectSM);

    // instantiate vision processor that needs to learn
    ImageProcessor learningProc = Vision.availableValidationProcessors.getInstance(this, learnerDetail.getImageProcessorType());

    // set up notification chain (detection SM -> learner -> detector/validator to learn)
    // NOTE: it's unclear what the typeId should be here. typeId == -1 doesn't work because the vision pipeline currently
    // doesn't send notifications with a typeId != -1 to registered procs with typeId -1
    // but -1 probably makes the most sense because learning isn't part of any particular visual search
    objectSM.getLastVisionProcessor().registerForNotification(learner, objectSM.getTypeId());//-1L);
    learner.registerForNotification(learningProc, objectSM.getTypeId());//-1L);

    // start everything
    learningProc.start(objectSM);
    learner.start(objectSM);
    if (!objectSM.hasStarted(this)) {
      objectSM.start(this);
    }

    // stop things once the learner has performed one iteration
    Util.Sleep(2000);
    while (!learningProc.hasLearned()) {
      log.debug("Waiting on learner to learn...");
      Util.Sleep(50);
    }
    log.debug("done learning. Un-registering for notifications and stopping learning processors.");
    objectSM.getLastVisionProcessor().unregisterForNotification(learner, objectSM.getTypeId());//-1L);
    learner.unregisterForNotification(learningProc, objectSM.getTypeId());//-1L);
    learner.stop(objectSM, false);
    learningProc.stop(objectSM, true);
    learningProc.resetHasLearned();

    // add learned thing to availableValidationProcessors
    Vision.availableValidationProcessors.addImageProcessorOption(nameToLearn, learnerDetail.getImageProcessorType());

    // check if thing to learn is a POWER ref (e.g., "instanceOf(objects_3, knife)"),
    // add property to the objectRef,
    // get/create the detector/validator capable of detecting the objectRef,
    // (re)set typeId for objectRef,
    // and start search for object to continue tracking thing that was just learned
    if (PredicateHelper.isObjectRef(targetObjectDescriptors)) {
      Symbol objectRef = new Symbol(targetObjectDescriptors.get(0).getName());

      // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
      Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), Vision.consultant.getVariable(objectRef));
      List<Term> nameToLearnList = Arrays.asList(nameToLearnMatchingVar);
      //List<Property> newProperties = Arrays.asList(new PredicateProperty(nameToLearnMatchingVar));
      Vision.consultant.assertProperties(objectRef, nameToLearnList);

      // ======================== START POWER HACK ================================
      // remove "object(X)" descriptor -- this is to enable learning of multiple objects so that in
      // subsequent "the object is a ..." utterances, "the object" doesn't get resolved to the object being
      // learned here, and instead will start a new "object" search
      List<Term> oldProperties = Arrays.asList(new Predicate("object", Vision.consultant.getVariable(objectRef)));
      Vision.consultant.retractProperties(objectRef, oldProperties);

      // check if this type has already been learned, and if so, remove duplicate ref
      List<Symbol> matchingRefs = Vision.consultant.getReferences(Vision.consultant.getAssertedProperties(objectRef));
      if (matchingRefs.size() > 1) {
        Vision.consultant.removeReference(objectRef);
      }
      // ======================== END POWER HACK ================================

//      Long typeId = Vision.consultant.getTypeId(objectRef);
//      SearchManager searchManager = Vision.availableSearchTypes.getInstance(this, typeId);
//      if (!searchManager.hasStarted(this)) {
//        searchManager.start(this);
//        while (!searchManager.hasIterationCompleted()) {
//          Util.Sleep(100);
//        }
//      }
//      if (objectSM.hasStarted(this)) {
//        objectSM.stop(this, false);
//      }
    }
  }

  private void learnDefinitionOf(Term learningTerm, Term nameToLearn, List<Term> targetObjectDescriptors) {
    log.debug("[learnDefinitionOf]: " + learningTerm);
    // TODO: figure out a way to to learn this type using new learning framework
//      LearnerDetail learnerDetail = typesDetail.get(LearnerType.DEFINITION);
//      Learner learner = new Learner(learnerDetail);
//      learner.setSingleIteration(true);
//      learner.addProcessingDescriptor(learningTerm, null);
//      learner.start(null);

    // TODO: add learned thing to availableValidationProcessors
//      Vision.availableValidationProcessors.

    Vision.availableSearchTypes.nameDescriptors(targetObjectDescriptors, nameToLearn);

    // check if thing to learn is a POWER ref (e.g., "definitionOf(objects_3, knife)") and add property to it
    if (PredicateHelper.isObjectRef(targetObjectDescriptors)) {
      Symbol objectRef = new Symbol(targetObjectDescriptors.get(0).getName());

      // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
      Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), Vision.consultant.getVariable(objectRef));
      List<Term> nameToLearnList = Arrays.asList(nameToLearnMatchingVar);
      //List<Property> properties = Arrays.asList(new PredicateProperty(nameToLearnMatchingVar));
      Vision.consultant.assertProperties(objectRef, nameToLearnList);
    } else {
      // find any existing POWER refs that have targetObjectDescriptors and add property nameToLearn to them
      for (Symbol objectRef : Vision.consultant.getReferences(targetObjectDescriptors)) {
        // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
        Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), Vision.consultant.getVariable(objectRef));
        List<Term> nameToLearnList = Arrays.asList(nameToLearnMatchingVar);
        //List<Property> properties = new ArrayList<>();
        //properties.add(new PredicateProperty(nameToLearnMatchingVar));
        Vision.consultant.assertProperties(objectRef,nameToLearnList);
      }
    }
  }

  private void unlearnDefinitionOf(Term learningTerm, Term nameToLearn, List<Term> targetObjectDescriptors) {
    log.debug("[unlearnDefinitionOf]: " + learningTerm);
    // TODO: figure out a way to to learn this type using new learning framework

    Vision.availableSearchTypes.removeNamedDescriptors(targetObjectDescriptors, nameToLearn);

    // remove property from any relevant POWER refs
    if (PredicateHelper.isObjectRef(targetObjectDescriptors)) {
      // if thing to unlearn is a POWER ref (e.g., "definitionOf(objects_3, knife)") and remove property from it
      Symbol objectRef = new Symbol(targetObjectDescriptors.get(0).getName());

      // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
      Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), Vision.consultant.getVariable(objectRef));
      List<Term> properties = new ArrayList<>();
      properties.add(nameToLearnMatchingVar);
      Vision.consultant.retractProperties(objectRef, properties);
    } else {
      // find any existing POWER refs that have targetObjectDescriptors and nameToLearn and remove nameToLearn from them
      List<Term> properties = new ArrayList<>();
      properties.addAll(targetObjectDescriptors);
      properties.add(new Predicate(nameToLearn));
      for (Symbol objectRef : Vision.consultant.getReferences(properties)) {
        // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
        properties.clear();
        Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), Vision.consultant.getVariable(objectRef));
        properties.add(nameToLearnMatchingVar);
        Vision.consultant.retractProperties(objectRef, properties);
      }
    }
  }
}
