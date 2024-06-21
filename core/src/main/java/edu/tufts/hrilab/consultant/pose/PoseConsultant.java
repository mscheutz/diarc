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

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return localConvertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    // get underlying pose info based on refId
    PoseReference ref = getReference(refId);
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
  public String toString() {
    return getReferenceSummaries();
  }

  public long calculateDistance(Symbol ref1, Symbol ref2) {
    Point3d p1 = getReference(ref1).getPosition(); //fixme: Unsafe operations
    Point3d p2 = getReference(ref2).getPosition();
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
      Symbol refId = Factory.createSymbol(ref.getRefId());
      if (getReference(refId) != null) {
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
      jsonClass.poseReferences.addAll(getAllReferences().stream().map(PoseReferenceJson::new).collect(Collectors.toList()));
      gson.toJson(jsonClass, writer);
      writer.flush();
    } catch (IOException e) {
      log.error("Trying to write vision references json to file.", e);
      return false;
    }

    return true;
  }
}
