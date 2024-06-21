/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.scene.SceneCollection;
import edu.tufts.hrilab.vision.VisionInterface;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;

import java.util.*;

public class MockVisionConsultant extends VisionConsultant {

  private SceneCollection sceneCollection;

  public MockVisionConsultant(VisionInterface visionComponent, String kbName, SceneCollection sceneCollection) {
    super(visionComponent,kbName);
    this.sceneCollection = sceneCollection;
    addPropertiesHandled(this.sceneCollection.getProperties());
  }

  // TODO: this method is potentially problematic in cases where more than one token exists for
  // a given objectRef, but maybe that's expected given that POWER (ref res) can't handle plurals yet.
  @Override
  public Map<Symbol, Double> getActivatedEntities() {
    // get all tokens in "STM"
    // mock vision doesn't currently have a STM, so just returning all objects in the scene
    List<MemoryObject> tokens = sceneCollection.getSceneDetectionResults();

    // filter scene graph tokens based on active visual searches
    List<MemoryObject> filteredTokens = new ArrayList<>();
    for (MemoryObject token : tokens) {
      for (Long typeId : visionComponent.getTypeIds()) {
        List<Term> descriptors = visionComponent.getDescriptors(typeId);
        if (descriptors.size() == 1 && descriptors.get(0).getName().equals("any")) {
          // use all tokens in this case
          token.setTypeId(typeId);
          filteredTokens.add(token);
        } else if (edu.tufts.hrilab.fol.util.Utilities.containsAllPredicates(new ArrayList<>(MemoryObjectUtil.getSceneGraphDescriptors(token)), descriptors)) {
          // token description matches typeId -- use token in this case
          token.setTypeId(typeId);
          filteredTokens.add(token);
        } else {
          log.debug("[getActivatedEntities] ignoring token: " + token);
        }
      }
    }

    // get/generate the objectRefs
    Map<Symbol, Double> activatedEntities = new HashMap<>();
    for (MemoryObject token : filteredTokens) {
      // first try to find matching objectRef for token
      Symbol objectRef = null;
      if (visionTypes.containsKey(token.getTypeId())) {
        for (VisionReference visionRef : visionTypes.get(token.getTypeId())) {
          if (visionRef.tokenIds.contains(token.getTokenId())) {
            objectRef = visionRef.refId;
          }
        }
      }

      // if objectRef wasn't found -- create new one
      if (objectRef == null) {
        objectRef = getNextReferenceId();
        List<Long> tokenIds = Arrays.asList(token.getTokenId());
        VisionReference visionRef = new VisionReference(objectRef, token.getVariable(), token.getTypeId(), tokenIds, new ArrayList<>());
        addReference(visionRef);
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

    log.debug("activated entities: "+activatedEntities);
    return activatedEntities;
  }

  @Override
  public Double process(Term constraint, Map<Variable, Symbol> bindings) {
    log.debug("Being asked how probable it is that " + constraint + " holds under bindings " + bindings);

    // get any objectRef -- all of them should have full set of properties, including the constraint
    Symbol objectRef = (Symbol) bindings.values().toArray()[0];
    VisionReference visionRef = getReference(objectRef);
    if(visionRef != null){
      log.debug("properties: "+visionRef.properties);
      log.debug("constraint: "+constraint);
      log.debug("contains: "+visionRef.properties.stream().filter(refProp -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(refProp, constraint)).toArray().length);
    }
    // TODO: what to do if more than one token for objectRef ?
    MemoryObject token=null;
    if(visionRef !=null && !visionRef.tokenIds.isEmpty()){
      token =  visionComponent.getToken(visionRef.tokenIds.get(0));
    }

    log.debug("token: "+token);
    if (token == null) {
      //TODO:brad: convert to allMatch? anyMatch?
      if (visionRef != null && visionRef.properties.stream().filter(refProp -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(refProp, constraint)).toArray().length > 0) {
        return 1.0;
      }else {
        return 0.0;
      }
    }


    for (Term moProperty : MemoryObjectUtil.getSceneGraphDescriptors(token)) {
      if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(moProperty, constraint)) {
        log.debug("visionRef " + objectRef + " contains property: " + constraint);

        // add constraint to STM-buffer
        visionRef.properties.add(constraint);
        return 1.0;
      }
    }

    // check against default vision consultant properties (e.g., it, that, ...)
    for (Term defaultProperty : defaultProperties) {
      if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(defaultProperty, constraint)) {
        return 1.0;
      }
    }

    log.debug("visionRef does NOT contains property: " + constraint);
    return 0.0;
  }
}
