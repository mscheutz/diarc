/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.consultant.pose;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.mtracs.util.MPose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MPoseConsultant extends Consultant<MPoseReference> {
  public MPoseConsultant(Class<MPoseReference> refClass, String kbName) {
    super(refClass, kbName);
  }

  public MPoseConsultant(Class<MPoseReference> refClass, String kbName, List<String> properties) {
    super(refClass, kbName, properties);
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type) {
    return localConvertToType(refId, type, new ArrayList<>());
  }

  @Override
  public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    // get underlying pose info based on refId
    MPoseReference ref = getReference(refId);
    MPose pose = ref.pose;

    if (type.isAssignableFrom(MPose.class)) {
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

  public Symbol updateReference(Term uniqueProperty, MPose newPose){
    Symbol found=null;

    for(MPoseReference ref: getAllReferences()){
      if(ref.properties.contains(uniqueProperty)){
        found = ref.refId;
        break;
      }
    }

    if(found != null) {
      addReference(new MPoseReference(found,(Variable) uniqueProperty.get(0),newPose, Arrays.asList(new Term []{uniqueProperty})));
      return found;
    }else{
      Symbol newRefID=getNextReferenceId();
      addReference(new MPoseReference(newRefID,(Variable) uniqueProperty.get(0),newPose, Arrays.asList(new Term []{uniqueProperty})));
      log.warn("[updateReference] this probably shouldn't happen, updateReference called with property that doesn't exist in consultant: "+uniqueProperty+" adding a new reference");
      return newRefID;
    }
  }

}
