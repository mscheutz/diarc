/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant.pose;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.tufts.hrilab.util.Convert;
import edu.tufts.hrilab.vision.stm.Grasp;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Writer;
import java.util.*;
import java.util.stream.Collectors;

public class PoseConsultant extends Consultant<PoseReference> {

  private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  public PoseConsultant(Class<PoseReference> refClass, String kbName, List<String> properties) {
    super(refClass, kbName, properties);
  }

  //todo: does not handle general race conditions on ref management across consultants.

  /**
   * Generates the new refId based on the refNumber counter, and increments. Informs all other
   * consultants with the same kbName of the current ref being allocated to attempt to maintain sync
   * and avoid duplicating refs.
   *
   * @return
   */
  @Override
  protected Symbol getNextReferenceId() {
    Symbol newReferenceId = Factory.createSymbol(kbName + "_" + refNumber + ":" + kbName);
    Collection<TRADEServiceInfo> consultantServices = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getKBName").returnType(String.class));
    for (TRADEServiceInfo consultantService : consultantServices) {
      try {
        String kbName = consultantService.call(String.class);
        if (kbName.equals(this.kbName)) { //this can be removed with proper filtering
          TRADE.getAvailableService(new TRADEServiceConstraints().name("externalUpdateReferenceNumber").inGroups(consultantService.getGroups().toArray(new String[0]))).call(void.class,refNumber);
        }
      } catch (TRADEException e) {
        throw new RuntimeException(e);
      }
    }
    return newReferenceId;
  }

  /**
   * Handles updating the refNumber when there are multiple consultants which could be generating new references.
   * Takes the current reference being allocated from the external consultant, and increments its current ref, assuming
   * the passed current ref has been allocated. Logs an error if the external current ref is less than the internal current ref,
   * which indicates that the local consultant value got updated at some point when the external consultant was missed.
   * Does not handle general race conditions wrt simultanous allocations, etc.
   *
   * @param externalCurrentRefNumber
   */
  @TRADEService
  public void externalUpdateReferenceNumber(int externalCurrentRefNumber) {
    //todo: will catch some cases where ref has already been created. does not handle actual race conditions.
    if (externalCurrentRefNumber < refNumber) {
      log.error("[externalUpdateReferenceNumber] external " + kbName + " consultant allocating reference number " + externalCurrentRefNumber + " but internal refNumber is " + refNumber);
    } else {
      refNumber = ++externalCurrentRefNumber;
    }
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return localConvertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    // get underlying pose info based on refId
    PoseReference ref = references.get(refId);
    Point3d location = ref.position;
    Quat4d orientation = ref.orientation;

    if (type.isAssignableFrom(PoseReference.class)) {
      //PoseReference
      return type.cast(ref);
    } else if (type.isAssignableFrom(Matrix4d.class)) {
      //Matrix4d
      Matrix4d mat = Convert.convertToMatrix4d(location, orientation);
      return type.cast(mat);
    } else if (type.isAssignableFrom(Point3d.class)) {
      // Point3d
      return type.cast(location);
    } else if (type.isAssignableFrom(Grasp.class)) {
      // Grasp
      Grasp grasp = new Grasp(location, orientation);
      return type.cast(grasp);
    } else if (type.isAssignableFrom(Grasp[].class)) {
      // Grasp[]
      Grasp grasp = new Grasp(location, orientation);
      Grasp[] grasps = {grasp};
      return type.cast(grasps);
    } else {
      log.error("[convertToType] Can not handle conversions to type: " + type);
      return null;
    }
  }

  @Override
  public String getReferenceSummaries() {
    StringBuilder sb = new StringBuilder();
    for (Symbol refId : references.keySet()) {
      sb.append(refId).append(" = ").append(references.get(refId).properties).append((references.get(refId).position == null) ? " (is null)" : " (not null)").append("\n");
    }
    return sb.toString();
  }

  @Override
  public PoseReference getReference(Symbol refId) {
    PoseReference localReference = references.get(refId);
    if (localReference != null) {
      return localReference;
    } else {
      //todo: assumes that reference sharing between all consultants with the same kbname is acceptable. what if there are multiple maps?
      Collection<TRADEServiceInfo> consultantServices = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getKBName").returnType(String.class));
      for (TRADEServiceInfo consultantService : consultantServices) {
        try {
          String kbName = consultantService.call(String.class);
          if (kbName.equals(this.kbName)) { //this can be removed with proper filtering
            PoseReference ref = TRADE.getAvailableService(new TRADEServiceConstraints().name("getReference").inGroups(consultantService.getGroups().toArray(new String[0]))).call(PoseReference.class,refId);
            if (ref != null) {
              return ref;
            }
          }
        } catch (TRADEException e) {
          throw new RuntimeException(e);
        }
      }
    }
    return null;
  }

  @Override
  public String toString() {
    return getReferenceSummaries();
  }

  public long calculateDistance(Symbol ref1, Symbol ref2) {
    Point3d p1 = references.get(ref1).getPosition(); //fixme: Unsafe operations
    Point3d p2 = references.get(ref2).getPosition();
    return Math.round(Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2) + Math.pow(p1.z - p2.z, 2)));
  }

  /**
   * Simplified Pose Reference class only used for reading/writing JSON files.
   */
  class PoseRefJson {
    public List<PoseReferenceJson> poseReferences;
  }

  /**
   * Load pre-defined pose references from file via JSON.
   *
   * @param filename
   */
  public void loadReferencesFromFile(String filename) {
    InputStream stream = PoseConsultant.class.getResourceAsStream(filename);
    if (stream == null) {
      log.error("Resource not found: " + filename);
      return;
    }

    BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
    PoseRefJson loadedPoseRefs = gson.fromJson(reader, PoseRefJson.class);
    log.debug("Parsed pose references from json:\n" + gson.toJson(loadedPoseRefs));

    for (PoseReferenceJson ref : loadedPoseRefs.poseReferences) {
      //todo: this actually increments the reference counter when doing this check. is there a way to make this fail more clearly? the below pattern feels convoluted
      getNextReferenceId();
      Symbol refId = Factory.createSymbol(ref.getRefId());
      if (references.containsKey(refId)) {
        log.error("[loadReferencesFromFile] reference " + refId + " already exists. Not loaded from file: " + ref);
        continue;
      }

      // update reference counters
      addReference(
              new PoseReference(
                      refId,
                      Factory.createVariable(ref.getVariable()),
                      ref.getProperties().stream().map(Factory::createPredicate).collect(Collectors.toList()),
                      ref.getPosition(), ref.getOrientation())
      );
    }
  }

  /**
   * Write all references to file.
   *
   * @param filename
   * @return if write was successful
   */
  public boolean writeReferencesToFile(String filename) {
    try {
      Writer writer = new FileWriter(filename);

      PoseRefJson jsonClass = new PoseRefJson();
      jsonClass.poseReferences = new ArrayList<>();
      jsonClass.poseReferences.addAll(references.values().stream().map(PoseReferenceJson::new).collect(Collectors.toList()));
      gson.toJson(jsonClass, writer);
      writer.flush();
    } catch (IOException e) {
      log.error("Trying to write vision references json to file.", e);
      return false;
    }

    return true;
  }
}
