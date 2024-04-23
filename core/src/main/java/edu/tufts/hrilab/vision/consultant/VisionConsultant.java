/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.consultant.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.vision.VisionInterface;
import edu.tufts.hrilab.vision.stm.Grasp;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import javax.vecmath.Point3d;
import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Writer;
import java.util.*;
import java.util.stream.Collectors;

public abstract class VisionConsultant extends Consultant<VisionReference> implements VisionConsultantInterface {
  /**
   * Map from typeId to VisionReference.
   */
  protected final Map<Long, Set<VisionReference>> visionTypes = new HashMap<>();
  /**
   * To give the consultant access to vision api methods and the STM.
   */
  protected VisionInterface visionComponent;
  /**
   * For reading/writing references to/from file.
   */
  private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  public VisionConsultant(VisionInterface visionComponent, String kbName) {
    super(VisionReference.class, kbName);
    this.visionComponent = visionComponent;
    this.defaultProperties = Arrays.asList(
            Factory.createPredicate("it", new Variable("X", kbName)),
            Factory.createPredicate("this", new Variable("X", kbName)),
            Factory.createPredicate("that", new Variable("X", kbName)),
            Factory.createPredicate("thing", new Variable("X", kbName)),
            Factory.createPredicate("those", new Variable("X", kbName)),
            Factory.createPredicate("they", new Variable("X", kbName)),
            Factory.createPredicate("these", new Variable("X", kbName))
    );
  }

  /**
   * Helper class to serialize/deserialize vision references to/from file via JSON.
   */
  class JsonClass {
    public List<VisionReferenceJson> visionReferences;
  }

  /**
   * Write vision references to file using JSON.
   *
   * @param filename
   * @return
   */
  public boolean writeReferencesToFile(String filename) {
    try {
      Writer writer = new FileWriter(filename);

      JsonClass jsonClass = new JsonClass();
      jsonClass.visionReferences = new ArrayList<>();
      jsonClass.visionReferences.addAll(references.values().stream().map(VisionReferenceJson::new).collect(Collectors.toList()));
      gson.toJson(jsonClass, writer);
      writer.flush();
    } catch (IOException e) {
      log.error("Trying to write vision references json to file.", e);
      return false;
    }

    return true;
  }

  /**
   * Load pre-defined vision references from file via JSON.
   *
   * @param filename
   */
  public void loadReferencesFromFile(String filename) {
    InputStream stream = VisionConsultant.class.getResourceAsStream(filename);
    if (stream == null) {
      log.error("Resource not found: " + filename);
      return;
    }

    BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
    JsonClass loadedVisionRefs = gson.fromJson(reader, JsonClass.class);
    log.debug("Parsed vision references from json:\n" + gson.toJson(loadedVisionRefs));
    loadedVisionRefs.visionReferences.forEach(ref -> addReference(
            new VisionReference(
                    Factory.createSymbol(ref.refId),
                    Factory.createVariable(ref.variable),
                    ref.properties.stream().map(Factory::createPredicate).collect(Collectors.toList())
            )
    ));
  }

  @Override
  public boolean assertProperties(Map<Variable, Symbol> bindings, Double prob, List<Term> properties) {
    // first convert properties to vision format
    // (i.e., unnested with free variables, e.g., on(grasp_point, VAR0) --> on(VAR3, VAR0), grasp_point(VAR3))

    for (Variable var : bindings.keySet()) {
      // add all properties to all refs in the bindings
      if (!assertProperties(bindings.get(var), properties)) {
        return false;
      }
    }

    return true;
  }

  @Override
  public boolean assertProperties(Symbol refId, List<Term> properties) {
    log.debug("[assertProperties] refId: " + refId + " properties: " + properties);

    if(!refId.hasType()) {
      refId = Factory.createSymbol(refId.getName(), refId.getName().split("_")[0]);
    }
    Reference ref = references.get(refId);
    if (ref == null) {
      log.error("[assertProperties] couldn't find existing ref: " + refId);
      return false;
    }

    // combine new properties with existing properties and convert them to vision form
    // combining with existing properties helps ensure variable consistency
    List<Term> descriptors = new ArrayList<>();
    descriptors.addAll(properties);
    descriptors.addAll(ref.properties);
    List<Term> convertedDescriptors = PredicateHelper.convertToVisionForm(descriptors);

    log.debug("[assertProperties] refId: " + refId + " pre assert ref properties: " + ref.properties);

    // update visionRef's properties
    ref.properties.clear();
    ref.properties.addAll(convertedDescriptors);

    log.debug("[assertProperties] refId: " + refId + " post assert ref properties: " + ref.properties);
    return true;
  }

  @Override
  public List<Symbol> getInitialDomain(List<Term> query) {
    log.debug("[getInitialDomain] " + query);
    // convert query Properties to Predicates
    log.debug("[getInitialDomain] vision refs: " + references.values());
    List<Symbol> objectRefs = new ArrayList<>();

    if (query.isEmpty()) {
      return new ArrayList<>(references.keySet());
    } else {
      for (Symbol objectRef : references.keySet()) {
        // compare query with objectRef properties
        //TODO: temporary fix, domain wasn't getting filled; figure out why
        if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(query, references.get(objectRef).properties)) {
          objectRefs.add(objectRef);
        }
      }

      log.debug("[getInitialDomain] query properties: " + query + " returned objectRefs: " + objectRefs);
      return objectRefs;
    }
  }

  public void setTypeId(Symbol objectRef, Long typeId) {
    VisionReference visionRef = references.get(objectRef);

    // remove from visionTypes container as old typeId
    Set<VisionReference> referencesByType = visionTypes.get(visionRef.typeId);
    if (referencesByType != null && referencesByType.contains(visionRef)) {
      referencesByType.remove(visionRef);
    }

    // (re)set typeId
    visionRef.typeId = typeId;

    // and add it to visionTypes container
    referencesByType = visionTypes.get(typeId);
    if (referencesByType == null) {
      referencesByType = new HashSet<>();
      visionTypes.put(typeId, referencesByType);
    }
    referencesByType.add(visionRef);
  }

  @Override
  public Long getTypeId(Symbol objectRef) {
    log.debug("[getTypeId] objectRef: " + objectRef);
    if(!objectRef.isTerm() && !objectRef.hasType()) {
      objectRef = Factory.createSymbol(objectRef.getName() + ":" + objectRef.getName().split("_")[0]);
    }
    // if objectRef is actually a reference resolution id
    if (!objectRef.isTerm() && objectRef.getName().startsWith(kbName)) {
      VisionReference visionRef = references.get(objectRef);
      if (visionRef == null) {
        log.error("[getTypeId] vision has not created object ref: " + objectRef);
        return -1L;
      } else {

        // if typeId hasn't been created for object ref, try to create one (by instantiating a new search for its properties)
        if (visionRef.typeId == -1L || !visionComponent.getAvailableTypeIds().contains(visionRef.typeId)) {
          Long typeId = visionComponent.getTypeId(visionRef.properties);

          // FIXME
          if (typeId != -1L) {
            visionRef.typeId = typeId;
            if (visionTypes.containsKey(visionRef.typeId)) {
              visionTypes.get(typeId).add(visionRef);
            } else {
              Set<VisionReference> refs = new HashSet<>();
              refs.add(visionRef);
              visionTypes.put(typeId, refs);
            }
          } else {
            log.debug("[getTypeId] (objectRef) couldn't create search.");
          }
        }

        return visionRef.typeId;
      }
    } else {
      // otherwise it's a descriptor
      List<Symbol> descriptors = new ArrayList<>();
      descriptors.add(objectRef);
      return visionComponent.getTypeId(descriptors);
    }
  }

  @Override
  public List<MemoryObject> getTokens(Symbol objectRef) {
    log.debug("[getTokens] objectRef: " + objectRef);
    // if objectRef is actually a reference resolution id
    if (!objectRef.isTerm() && objectRef.getName().startsWith(kbName)) {
      VisionReference visionRef = references.get(objectRef);
      // With object permanance changes, tokenId list for any objectRef should be of length 1
      // In this case we don't want to check to add the tokenId to the visionRef, because it
      //   should always already be there
      // Create and return new MO list just to match current interface using symbol
      // EAK: this relies on some other mechanism to update visionRefs with native MO results
//      List<MemoryObject> tokens = new ArrayList();
//      for (Long tokenId : visionRef.tokenIds) {
//        MemoryObject token = visionComponent.getToken(tokenId, 0.0);
//        if (token != null) {
//          tokens.add(token);
//        }
//      }
      List<MemoryObject> tokens = visionComponent.getTokens(visionRef.typeId, 0.0);
      for (MemoryObject token : tokens) {
        if (!visionRef.tokenIds.contains(token.getTokenId())) {
          visionRef.tokenIds.add(token.getTokenId());
        }
      }
      log.debug("got tokens");
      return tokens;
    } else {
      // otherwise it's a descriptor
      List<Symbol> descriptors = new ArrayList();
      descriptors.add(objectRef);
      return visionComponent.getTokens(descriptors, 0.0f);
    }
  }

  @Override
  public List<Long> getTokenIds(Symbol objectRef) {
    if(!objectRef.hasType()) {
      objectRef = Factory.createSymbol(objectRef.getName() + ":" + objectRef.getName().split("_")[0]);
    }
    log.debug("[getTokenIds] objectRef: " + objectRef);
    // if objectRef is actually a reference resolution id
    if (!objectRef.isTerm() && objectRef.getName().startsWith(kbName)) {
      VisionReference visionRef = references.get(objectRef);
      // With object permanance changes, tokenId list for any objectRef should be of length 1
      // In this case we don't want to check to add the tokenId to the visionRef, because it
      //   should always already be there
      // Create and return new MO list just to match current interface using symbol
      // EAK: this relies on some other mechanism to update visionRefs with native MO results
//      List<Long> tokenIds = new ArrayList();
//      for (Long tokenId : visionRef.tokenIds) {
//        MemoryObject token = visionComponent.getToken(tokenId, 0.0);
//        if (token != null) {
//          tokenIds.add(tokenId);
//        }
//      }
      // TODO: EAK: this is inconsistent with how getTokens works, and should work the same way
      List<Long> tokenIds = visionComponent.getTokenIds(visionRef.typeId, 0.0);
      for (Long tokenId : tokenIds) {
        if (!visionRef.tokenIds.contains(tokenId)) {
          visionRef.tokenIds.add(tokenId);
        }
      }
      return tokenIds;
    } else {
      // otherwise it's a descriptor
      List<Symbol> descriptors = new ArrayList();
      descriptors.add(objectRef);
      return visionComponent.getTokenIds(descriptors, 0.0f);
    }
  }

  /**
   * This method should probably be removed. Only added for object learning hack in AvailableLearners.
   *
   * @param objectRef
   * @return
   */
  public boolean removeReference(Symbol objectRef) {
    if(!objectRef.hasType()) {
      objectRef = Factory.createSymbol(objectRef.getName() + ":" + objectRef.getName().split("_")[0]);
    }
    if (references.containsKey(objectRef)) {
      references.remove(objectRef);
      return true;
    } else {
      return false;
    }
  }

  /**
   * Get POWER reference for vision type ID.
   *
   * @param typeId
   * @return
   */
  public List<Symbol> getReferences(Long typeId) {
    if (visionTypes.containsKey(typeId)) {
      List<Symbol> objectRefs = new ArrayList<>();
      for (VisionReference visionRef : visionTypes.get(typeId)) {
        objectRefs.add(visionRef.refId);
      }
      return objectRefs;
    } else {
      log.error("[getReference] no reference for typeId: " + typeId);
      return null;
    }
  }

  /**
   * Get all POWER references known to vision.
   *
   * @return
   */
  public List<Symbol> getReferences() {
    return new ArrayList<>(references.keySet());
  }

  /**
   * Get POWER reference that contain all specified properties.
   *
   * @param properties
   * @return
   */
  public List<Symbol> getReferences(List<Term> properties) {
    List<Symbol> objectRefs = new ArrayList<>();

    for (VisionReference visionRef : references.values()) {
      if (edu.tufts.hrilab.fol.util.Utilities.containsAllPredicates(visionRef.properties, properties)) {
        objectRefs.add(visionRef.refId);
      }
    }
    return objectRefs;
  }

  /**
   * Get free-variable for specified object reference.
   *
   * @param objectRef
   * @return
   */
  public Variable getVariable(Symbol objectRef) {
    if (references.containsKey(objectRef)) {
      return references.get(objectRef).variable;
    } else {
      return null;
    }
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return convertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    List<MemoryObject> mos = getTokens(refId);
    if (mos == null || mos.isEmpty()) {
      log.debug("[getTargetType] no MemoryObject associated with refId: " + refId);
      return null;
    }

    // transform all MOs to base frame
    mos.forEach(MemoryObject::transformToBase);

    if (type.isAssignableFrom(MemoryObject.class)) {
      // MemoryObject
      return convertToType(mos.get(0), type);
    } else if (type.isAssignableFrom(MemoryObject[].class)) {
      // MemoryObject[]
      return convertToType(mos.toArray(new MemoryObject[0]), type);
    } else if (type.isAssignableFrom(Grasp.class)) {
      // Grasp
      List<Grasp> grasps = convertToGrasps(mos.get(0), constraints);
      return convertToType(grasps.get(0), type);
    } else if (type.isAssignableFrom(Grasp[].class)) {
      // Grasp[]
      List<Grasp> grasps = convertToGrasps(mos.get(0), constraints);
      return convertToType(grasps.toArray(new Grasp[0]), type);
    } else if (type.isAssignableFrom(Point3d.class)) {
      // Point3d
      return convertToType(mos.get(0).getLocation(), type);
    } else {
      log.error("[convertToType] Can not handle conversions to type: " + type);
      return null;
    }
  }

  protected List<Grasp> convertToGrasps(MemoryObject mo, List<? extends Term> constraints) {
    if (constraints == null || constraints.isEmpty()) {
      constraints = new ArrayList<>(MemoryObjectUtil.getSceneGraphDescriptors(mo));
    }

    List<Grasp> grasps = new ArrayList<>();

    // find variable name of "grasp_point" constraint
    Variable grasp_var = null;
    for (Term constraint : constraints) {
      if (constraint.getName().equals("grasp_point") && constraint.getOrderedVars().size() == 1) {
        grasp_var = constraint.getOrderedVars().get(0);
        break;
      }
    }
    if (grasp_var == null) {
      log.error("[moveTo(group,object,constraints)] must contain a \"grasp_point\" constraint.");
      return grasps;
    }

    // before we do anything, make sure the MemoryObject is in base_link frame
    mo.transformToBase();

    // get all the grasp options
    List<Map<Variable, MemoryObject>> bindings = new ArrayList<>();
    if (MemoryObjectUtil.getMemoryObjectBindings(mo, constraints, bindings)) {
      for (Map<Variable, MemoryObject> bindingsInstance : bindings) {
        Grasp grasp = MemoryObjectUtil.convertMemoryObjectToGrasp(bindingsInstance.get(grasp_var));
        grasps.add(grasp);
      }
    }

    return grasps;
  }

  protected <U> U convertToType(Object object, Class<U> type) {
    // convert return type to non-primitive type for easy type matching
    type = Utilities.primitiveToObject(type);

    if (type.isInstance(object)) {
      return type.cast(object);
    } else if (object == null) {
      return null;
    } else {
      log.error("Cannot convert object to type: " + type);
      return null;
    }
  }
}
