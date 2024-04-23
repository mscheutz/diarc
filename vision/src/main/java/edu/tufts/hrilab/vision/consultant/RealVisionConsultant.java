/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.VisionInterface;
import edu.tufts.hrilab.vision.stm.*;
import java.util.*;

public class RealVisionConsultant extends VisionConsultant {

  public RealVisionConsultant(VisionInterface visionComponent, String kbName) {
    super(visionComponent,kbName);
  }

  @Override
  public List<Term> getPropertiesHandled() {
    //TODO:brad; update this to use addProperties handled
    // first get all predicates from vision
    List<Term> predicates = new ArrayList<>();
    for (NamedDescription namedDesc : Vision.availableSearchTypes.getDefinitions()) {
      predicates.add(namedDesc.getName());
    }
    predicates.addAll(Vision.availableValidationProcessors.getOptions());
    predicates.addAll(Vision.availableDetectors.getOptions());
    // then add all default vision consultant properties (e.g., it, that, ...)
    predicates.addAll(defaultProperties);

    return predicates;
  }

  // TODO: this method is potentially problematic in cases where more than one token exists for
  // a given objectRef, but maybe that's expected given that POWER (ref res) can't handle plurals yet.
  @Override
  public Map<Symbol, Double> getActivatedEntities() {
    // get all tokens in STM
    List<MemoryObject> tokens = ShortTermMemoryInterface.getTokens(0.0);

    // get/generate the objectRefs
    Map<Symbol, Double> activatedEntities = new HashMap<>();
    for (MemoryObject token : tokens) {
      // first try to find matching objectRef for token
      Symbol objectRef = null;
      if (visionTypes.containsKey(token.getTypeId())) {
        // first search based on tokenId
        for (VisionReference visionRef : visionTypes.get(token.getTypeId())) {
          if (visionRef.tokenIds.contains(token.getTokenId())) {
            objectRef = visionRef.refId;
          }
        }

         // TODO: this is a HUGE assumption that only one object exists for a given set of properties
         // if objectRef wasn't found by tokenId -- search by typeId and properties
         if (objectRef == null) {
           for (VisionReference visionRef : visionTypes.get(token.getTypeId())) {
             if (edu.tufts.hrilab.fol.util.Utilities.containsAllPredicates(new ArrayList<>(MemoryObjectUtil.getSceneGraphDescriptors(token)),
                     visionRef.properties)) {
              objectRef = visionRef.refId;
             }
           }
         }
      }

      // if objectRef wasn't found -- create new one
      if (objectRef == null) {
        objectRef = getNextReferenceId();
        List<Term> properties = new ArrayList<>(MemoryObjectUtil.getSceneGraphDescriptors(token));

        if (log.isDebugEnabled()) {
          log.debug("[getActivatedEntities] existing visionTypes: " + visionTypes);
          log.debug("[getActivatedEntities] creating new objectRef " + objectRef + " with properties: " + properties);
          log.debug("[getActivatedEntities] creating new objectRef. Existing objectRefs: " + getReferenceSummaries());
          for (NamedDescription description : Vision.availableSearchTypes.getDefinitions()) {
            log.debug("[getActivatedEntities] creating new objectRef. Existing definitions: " + description.getName() + " = " + description.getDescriptors());
          }
        }

        List<Long> tokenIds = Arrays.asList(token.getTokenId());
        VisionReference visionRef = new VisionReference(objectRef, token.getVariable(), token.getTypeId(), tokenIds, properties);
        references.put(objectRef, visionRef);
        if (visionTypes.containsKey(token.getTypeId())) {
          visionTypes.get(token.getTypeId()).add(visionRef);
        } else {
          Set<VisionReference> visionRefs = new HashSet<>();
          visionRefs.add(visionRef);
          visionTypes.put(token.getTypeId(), visionRefs);
        }
      }

      // add tokens to POWER data structures in the case that the visual search wasn't started through POWER
      // NOTE: the double value here is a visual saliency value, not the detection confidence. this is used by Growler
      // to generate relevanceWeightings. eventually a more complex mechanism will be needed to deal with saliency
      activatedEntities.put(objectRef, 10.0);
    }

    return activatedEntities;
  }

  @Override
  public boolean assertProperties(Symbol objectRef, List<Term> properties) {
    super.assertProperties(objectRef, properties);

    // TODO: it might be better to modify the existing search manager with new properties instead of
    // staring a separate search and changing the typeId of the objectRef
    //
    // get visual search capable of detecting all objectRef's new properties, change the typeId of the objectRef, and
    // start the new search if the old search was
    VisionReference visionRef = references.get(objectRef);
    if (!visionRef.typeId.equals(-1L)) {
      SearchManager currSearchManager = Vision.availableSearchTypes.getInstance(this, visionRef.typeId);
      boolean isCurrentSearchRunning = currSearchManager.isRunning();

      List<Term> objProperties = Vision.consultant.getAssertedProperties(objectRef);
      Long typeId = visionComponent.getTypeId(objProperties);
      if (!typeId.equals(visionRef.typeId)) {
        Vision.consultant.setTypeId(objectRef, typeId);

        if (isCurrentSearchRunning) {
          // new search is already started by call to getTypeId
          long startTime = System.currentTimeMillis();
          long maxWaitTime = 5000;
          while (ShortTermMemoryInterface.getTokens(visionRef.typeId,0.0).isEmpty()) {
            if ((System.currentTimeMillis() - startTime) > maxWaitTime) {
              log.warn("[assertProperties] new search did not detect object within allotted timeframe.");
              break;
            }
            Util.Sleep(100);
          }

          // then stop the old search
          currSearchManager.stop(this, true);
        }
      }
    }

    return true;
  }

}
