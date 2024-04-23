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
import java.util.Map;

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
    MPoseReference ref = references.get(refId);
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
  public String getReferenceSummaries() {
    StringBuilder sb = new StringBuilder();
    for (Symbol refId : references.keySet()) {
      sb.append(refId).append(" = ").append(references.get(refId).properties).append((references.get(refId).pose == null)? " (is null)" : " (not null)").append("\n");
    }
    return sb.toString();
  }

  @Override
  public String toString() {
    return getReferenceSummaries();
  }

  public Symbol updateReference(Term uniqueProperty, MPose newPose){
    Symbol found=null;

    for(Map.Entry<Symbol, MPoseReference> ref: references.entrySet()){
      if(ref.getValue().properties.contains(uniqueProperty)){
        found= ref.getKey();
        break;
      }
    }

    if(found != null) {
      references.put(found, new MPoseReference(found,(Variable) uniqueProperty.get(0),newPose, Arrays.asList(new Term []{uniqueProperty})));
      return found;
    }else{
      Symbol newRefID=getNextReferenceId();
      references.put(newRefID,new MPoseReference(newRefID,(Variable) uniqueProperty.get(0),newPose, Arrays.asList(new Term []{uniqueProperty})));
      log.warn("[updateReference] this probably shouldn't happen, updateReference called with property that doesn't exist in consultant: "+uniqueProperty+" adding a new reference");
      return newRefID;
    }
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
//  @Override
//  protected Symbol getNextReferenceId() {
//    //todo: Now that this has type information, do we still care about kbname? -MF
//    Symbol newReferenceId = Factory.createSymbol(kbName + "_" + refNumber + ":" + kbName);
//    Set<TRADEServiceInfo> consultants = TRADE.getAvailable("getActivatedEntities", new String[0]);
//    for (TRADEServiceInfo consultant : consultants) {
//      try {
//        String kbName = (String) TRADE.callThe(consultant, "getKBName");
//        if (kbName.equals(this.kbName)) { //this can be removed with proper filtering
//          TRADE.callThe(consultant, "externalUpdateReferenceNumber", refNumber);
//        }
//      } catch (TRADEException e) {
//        throw new RuntimeException(e);
//      }
//    }
//    return newReferenceId;
//  }


  //todo: see getNextReferenceId. this is duplicate code.
  /**
   * Handles updating the refNumber when there are multiple consultants which could be generating new references.
   * Takes the current reference being allocated from the external consultant, and increments its current ref, assuming
   * the passed current ref has been allocated. Logs an error if the external current ref is less than the internal current ref,
   * which indicates that the local consultant value got updated at some point when the external consultant was missed.
   * Does not handle general race conditions wrt simultanous allocations, etc.
   *
   * @param externalCurrentRefNumber
   */
//  @TRADEService
//  public void externalUpdateReferenceNumber(int externalCurrentRefNumber) {
//    //todo: will catch some cases where ref has already been created. does not handle actual race conditions.
//    if (externalCurrentRefNumber < refNumber) {
//      log.error("[externalUpdateReferenceNumber] external " + kbName + " consultant allocating reference number " + externalCurrentRefNumber + " but internal refNumber is " + refNumber);
//    } else {
//      refNumber = ++externalCurrentRefNumber;
//    }
//  }
}
