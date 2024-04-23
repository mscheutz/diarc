/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.consultant.vision;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.ConsultantInterface;
import edu.tufts.hrilab.mtracs.util.CognexJob;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class CognexConsultant extends Consultant<CognexReference> implements ConsultantInterface {

    //TODO:brad: combine this with jobIndices
    protected Set<CognexJob> availableJobs;
    protected Map<String, CognexJob> boundJobs;

    /**
     * Group identifier to be prepended to reference names
     */
    protected String groupNames;

    public CognexConsultant(List<String> groups) {
        super(CognexReference.class, "physobj");
        StringBuilder sb = new StringBuilder();
        for (String group : groups) {
            sb.append(group.split(":")[1]);//todo: this wants the second portion of agent:robotone, and nothing else? hacky.
        }
        groupNames = sb.toString();

        propertiesHandled.add(Factory.createPredicate("hole(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("m3(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("deepM3(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("left(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("right(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("top(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("bottom(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("prop(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("bottle(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("nv30fau(X:" + getKBName() + ")"));
        propertiesHandled.add(Factory.createPredicate("nf32sv(X:" + getKBName() + ")"));
        //Jobs
        availableJobs = new HashSet<>();
        CognexJob foodJob = new CognexJob("food", "classifier");
        foodJob.addJob(Factory.createSymbol("milk"));
        foodJob.addJob(Factory.createSymbol("pb"));
        foodJob.addJob(Factory.createSymbol("peaches"));
        foodJob.addJob(Factory.createSymbol("fries"));
        foodJob.addJob(Factory.createSymbol("cereal"));
        foodJob.addJob(Factory.createSymbol("soda"));
        foodJob.addJob(Factory.createSymbol("oj"));
        foodJob.addJob(Factory.createSymbol("chicken"));
        foodJob.addJob(Factory.createSymbol("grapes"));
        availableJobs.add(foodJob);
        CognexJob holeM3Job = new CognexJob("holeM3", "detector");
        availableJobs.add(holeM3Job);
        CognexJob holeDeepJob = new CognexJob("holeDeep", "detector");
        availableJobs.add(holeDeepJob);
        CognexJob mountJob = new CognexJob("feedrDet", "detector");
        availableJobs.add(mountJob);
        CognexJob cbJob = new CognexJob("cbDet", "detector");
        availableJobs.add(cbJob);
        CognexJob nvJob = new CognexJob("nvDet", "detector");
        availableJobs.add(nvJob);
        CognexJob pillJob = new CognexJob("pillDet", "detector");
        availableJobs.add(pillJob);
        CognexJob ioCJOb = new CognexJob("ioCDet", "detector");
        availableJobs.add(ioCJOb);
        CognexJob sBoxJob = new CognexJob("sBoxDet", "detector");
        availableJobs.add(sBoxJob);
        CognexJob antisepticJob = new CognexJob("antisDet", "detector");
        availableJobs.add(antisepticJob);
        CognexJob propJob = new CognexJob("propDet", "detector");
        availableJobs.add(propJob);


        boundJobs = new HashMap<>();
        //TODO:might want to remove this if we teach all of the item and recipe definitions through an action script.
        boundJobs.put("screwHead", mountJob);
        boundJobs.put("nv30fau", nvJob);
        boundJobs.put("nf32sv", cbJob);
        boundJobs.put("pillBottle", pillJob);
        boundJobs.put("ioCard", ioCJOb);
        boundJobs.put("bandage", ioCJOb);
        boundJobs.put("screwBox", sBoxJob);
        boundJobs.put("antiseptic", antisepticJob);
        boundJobs.put("prop", propJob);
        boundJobs.put("bottle", propJob);
        boundJobs.put("m3Hole", holeM3Job);
        boundJobs.put("deepM3Hole", holeDeepJob);
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
            if (references.containsKey(refId)) {
                CognexReference ref = references.get(refId);
                ref.properties.addAll(properties);
                for (Term p : ref.properties) {
                    if (boundJobs.containsKey(p.getName())) {
                        ref.setCognexJob(boundJobs.get(p.getName()));
                        //TODO:brad: this is a shortcut for now. We would probably want some sort of function that takes in all of the properties and returns the job name. That way we could support classifiers that use Job and Index as descriptors.
                        break;
                    }
                }
            } else {
                return false;
            }
        }
            return true;
        }

    @Override
    public boolean assertProperties(Symbol refId, List<Term> properties) {
        CognexReference ref = references.get(refId);
        ref.properties.addAll(properties);
        for (Term p : ref.properties) {
            if (boundJobs.containsKey(p.getName())) {
                ref.setCognexJob(boundJobs.get(p.getName()));
                //TODO:brad: this is a shortcut for now. We would probably want some sort of function that takes in all of the properties and returns the job name. That way we could support classifiers that use Job and Index as descriptors.
                break;
            }
        }
        return true;
    }


    /**
     * Adds a binding between the human descriptor and the name of the job on the cognex, these doesn't necessarily need to be the same. The system should know about all of the job names, but it doesn't have to know about the human producible descriptors?
     * <p>
     * Once the binding is done it adds the property to the consultant. TODO:brad is there a way we could just get the jobs from the Cognex at start up? that would make this a lot easier.
     *
     * @param descriptor human provided descriptor
     * @param jobName    name of job on the Cognex that is used to detect the thing associated with the provided descriptor
     */
    @TRADEService
    public void addDetectionType(Symbol descriptor, Symbol jobName) {
        //add pose name to properties consultant can handle


        CognexJob job = null;
        for (CognexJob j : availableJobs) {
            if (j.getName().equals(jobName.getName())) {
                job = j;
                break;
            }
        }
        if (job == null) {
            log.error("[addDetectionType] no job found for id " + jobName);
            return;
        }
        List<Term> props = new ArrayList<>();
        props.add(Factory.createPredicate(descriptor.getName(), "X:" + getKBName()));
        //TODO:brad: we need to design a comprehensive mechanism to allows users to re fer to specific cognex jobs/ids within jobs
//        if (job.getType().equals("classifier")) {
//            props.add(Factory.createPredicate(index.toString(), "X"));
//        }
        addPropertiesHandled(props);
        boundJobs.put(descriptor.getName(), job);

//        String entry;
//        if (job.getType().equals("classifier")) {
//            entry = index.toString();
//        } else
//        if (job.getType().equals("detector")) {
//            entry = job.getName();
//        } else {
//            log.error("[addDetectionType] Unrecognized job type: " + job.getType());
//            return;
//        }

        //inform parser of new location
//        //TODO reimplement homophone permutations
//        try {
//            //TODO:brad generalize this, perhaps make it so you can pass in additional string modifications as an arg?
//            if (descriptor.getName().equals("m3")) {
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName(), "SCREWTYPE", descriptor.getName(), "");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName() + " screw", "SCREWTYPE", descriptor.getName(), "");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName() + " hole", "DESC", descriptor.getName(), "");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName() + " holes", "DESC", descriptor.getName(), "");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName() + " hole", "RN", descriptor.getName(), "VAR");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName() + " holes", "RN", descriptor.getName(), "VAR");
//            } else {
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName(), "DESC", descriptor.getName(), "");
//                TRADE.callThe("injectDictionaryEntry", descriptor.getName(), "RN", descriptor.getName(), "VAR");
//            }
//        } catch (TRADEException e) {
//            log.error("[addDetectionType] exception calling injectDictionaryEntry for descriptor " + descriptor, e);
//        }
    }

    public CognexReference createCognexRef(CognexJob job, List<Term> additionalProps) {

        //if there are any matching hypothetical bind them first
        for (CognexReference r : references.values()) {
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

    protected Symbol getNextReferenceId() {
        String kbName = this.kbName;
        return Factory.createSymbol( groupNames + kbName + "_" + this.refNumber++ + ":" + this.kbName);
    }

    /**
     * @param descriptor
     * @return
     */
    public CognexJob getJobForDescriptor(String descriptor) {
        return boundJobs.get(descriptor);
    }


    @TRADEService
    public CognexReference removeReference(Symbol refId) {
        if (this.references.containsKey(refId)) {
            return this.references.remove(refId);
        }
        return null;
    }

    public boolean insertReference(Symbol refId, CognexReference ref) {
        return (this.references.putIfAbsent(refId, ref) == null);
    }


//    //todo: does not handle general race conditions on ref management across consultants.
//    //todo: duplicates code in the diarc PoseConsultant. We didn't want to implement a non-general
//    //  version of reference sharing in the base Consultant class, but this is needed since
//    //  the reference counter is no longer static in the base class.
//    /**
//     * Generates the new refId based on the refNumber counter, and increments. Informs all other
//     * consultants with the same kbName of the current ref being allocated to attempt to maintain sync
//     * and avoid duplicating refs.
//     *
//     * @return
//     */
//    @Override
//    protected Symbol getNextReferenceId() {
//        //todo: Now that this has type information, do we still care about kbname? -MF
//        Symbol newReferenceId = Factory.createSymbol(kbName + "_" + refNumber + ":" + kbName);
//        Set<TRADEServiceInfo> consultants = TRADE.getAvailable("getActivatedEntities", new String[0]);
//        for (TRADEServiceInfo consultant : consultants) {
//            try {
//                String kbName = (String) TRADE.callThe(consultant, "getKBName");
//                if (kbName.equals(this.kbName)) { //this can be removed with proper filtering
//                    TRADE.callThe(consultant, "externalUpdateReferenceNumber", refNumber);
//                }
//            } catch (TRADEException e) {
//                throw new RuntimeException(e);
//            }
//        }
//        return newReferenceId;
//    }


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
//    @TRADEService
//    public void externalUpdateReferenceNumber(int externalCurrentRefNumber) {
//        //todo: will catch some cases where ref has already been created. does not handle actual race conditions.
//        if (externalCurrentRefNumber < refNumber) {
//            log.error("[externalUpdateReferenceNumber] external " + kbName + " consultant allocating reference number " + externalCurrentRefNumber + " but internal refNumber is " + refNumber);
//        } else {
//            refNumber = ++externalCurrentRefNumber;
//        }
//    }
}
