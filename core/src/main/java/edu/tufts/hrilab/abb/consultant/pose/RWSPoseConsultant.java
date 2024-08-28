/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.pose;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RWSPoseConsultant extends Consultant<RWSPoseReference> {

  Map<Symbol, Symbol> cameraPoseToDropoffPoseMap = new HashMap<>();

  private static String getGroupNamesString(List<String> groups) {
    StringBuilder sb = new StringBuilder();
    for (String group : groups) {
      sb.append(group.split(":")[1]);//todo: this wants the second portion of agent:robotone, and nothing else? hacky.
    }
    return sb.toString();
  }

  public RWSPoseConsultant(List<String> groups) {
    super(RWSPoseReference.class, getGroupNamesString(groups) + "pose");
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return localConvertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    // get underlying pose info based on refId
    RWSPoseReference ref = getReference(refId);
    String pose = ref.pose;

    if (type.isAssignableFrom(String.class)) {
      // Point3d
      return type.cast(pose);
    } else {
      log.error("[convertToType] Can not handle conversions to type: " + type);
      return null;
    }
  }

  @Override
  public String toString() {
    return getReferenceSummaries();
  }

  //TODO:brad implement file saving/loading
  public Symbol addCameraPose(String propertyName, String poseValue) {
    Variable v = Factory.createVariable("X", getKBName());
    Term property = Factory.createPredicate(propertyName, v);
    List<Term> properties = List.of(new Term[]{property});
    RWSPoseReference ref = new RWSPoseReference(getNextReferenceId(), v, properties, poseValue);
    addReference(ref);
    return ref.getRefId();
  }

  public Symbol addDropoffPose(String propertyName, String cameraPoseName, String poseValue) {
    Variable v = Factory.createVariable("X", getKBName());
    Term property = Factory.createPredicate(propertyName, v);
    List<Term> properties = List.of(new Term[]{property});
    RWSPoseReference ref = new RWSPoseReference(getNextReferenceId(), v, properties, poseValue);
    addReference(ref);
    for (RWSPoseReference tmpRef : getAllReferences()) {
      Term tmpProperty = Factory.createPredicate(cameraPoseName, v);
      if (tmpRef.properties.contains(tmpProperty)) {
        cameraPoseToDropoffPoseMap.put(tmpRef.refId, ref.getRefId());
      }
    }
    return ref.getRefId();
  }

  @Override
  protected Symbol getNextReferenceId() {
    return Factory.createSymbol(kbName + "_" + refNumber++ + ":" + kbName);
  }

  public boolean isCameraPoseRefId(Symbol refId) {
    return cameraPoseToDropoffPoseMap.containsKey(refId);
  }

  public Symbol getDropoffPoseForCameraPose(Symbol cameraPoseRefId) {
    Symbol toReturn = cameraPoseToDropoffPoseMap.get(cameraPoseRefId);
    if (toReturn == null) {
      log.error("[getDropoffPoseForCameraPose] no dropoff pose corresponding to camera pose " + cameraPoseRefId);
    }
    return toReturn;
  }

  //todo: refactor.
  public List<Symbol> getRefIdsWithPropertyNamed(String property) {
    List<Symbol> toReturn = new ArrayList<>();
    for (RWSPoseReference ref : getAllReferences()) {
      List<Term> properties = ref.properties;
      for (Term prop : properties) {
        if (prop.getName().equals(property)) {
          toReturn.add(ref.refId);
        }
      }
    }
    return toReturn;
  }
}
