/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.temiv3;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.*;

public class TemiV3PoseConsultant extends Consultant<TemiV3PoseReference> {
    protected static int poseRefNumber =0;
    List<Symbol> activatedEntities;

    public TemiV3PoseConsultant(Class<TemiV3PoseReference> refClass, String kbName, List<String> properties) {
        super(refClass, kbName, properties);
        this.activatedEntities = new ArrayList<>();
    }

    @Override
    protected Symbol getNextReferenceId() {
        //todo: Now that this has type information, do we still care about kbname? -MF
        return Factory.createSymbol(kbName + "_" + poseRefNumber++ + ":" + kbName);
    }

    @Override
    public <U> U localConvertToType(Symbol refId, Class<U> type) {
        return localConvertToType(refId, type, new ArrayList<>());
    }

    @Override
    public <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
        // get underlying pose info based on refId
        TemiV3PoseReference ref = getReference(refId);
        List<Float> pose = ref.rawPose;
        Symbol name = ref.getName();

        if (type.isAssignableFrom(Symbol.class)) {
            return type.cast(name);
        } else if (type.isAssignableFrom(ArrayList.class)) {
            return type.cast(pose);
        } else if (type.isAssignableFrom(String.class)) {
            return type.cast(name.toString());
        }
        else {
            log.error("[convertToType] Can not handle conversions to type: " + type);
            return null;
        }
    }

    @Override
    public String toString() {
        return getReferenceSummaries();
    }

    //Return explicitly tracked activated entities rather than all existing references
    @Override
    public Map<Symbol, Double> getActivatedEntities() {
        this.log.debug("[getActivatedEntities]");
        Map<Symbol, Double> vals = new HashMap<>();
        Iterator<Symbol> var2 = this.activatedEntities.iterator();

        while(var2.hasNext()) {
            Symbol ref = (Symbol)var2.next();
            vals.put(ref, 10.0D);
        }

        return vals;
    }

    //Anytime we assert properties to a refId (done in saveLocation), consider that reference as an activated entity
    @Override
    public boolean assertProperties(Map<Variable, Symbol> bindings, Double prob, List<Term> properties) {
        Iterator<Symbol> var4 = bindings.values().iterator();

        while(var4.hasNext()) {
            Symbol refId = var4.next();
            if (!this.activatedEntities.contains(refId)) {
                this.activatedEntities.add(refId);
            }
        }
        return super.assertProperties(bindings, prob, properties);
    }

    @Override
    public boolean assertProperties(Symbol refId, List<Term> properties) {
        if (!this.activatedEntities.contains(refId)) {
            this.activatedEntities.add(refId);
        }
        return super.assertProperties(refId, properties);
    }

    @Override
    public boolean addPropertiesHandled(List<Term> properties) {
        boolean propertiesAdded = true;
        for (Term property : properties) {
            if (!propertiesHandled.stream().anyMatch(p -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(p, property))) {
                propertiesHandled.add(property);
                //inform parser of new location
                log.debug("[addLocation] adding location " + property.getName() + " to parser dict");
                try {
                    TRADE.getAvailableService(new TRADEServiceConstraints().name("generateLocationRules").argTypes(String.class,String.class,Boolean.class)).call(void.class, property.getName(), getKBName(), false);
                } catch (TRADEException e) {
                    log.error("[addLocation] unable to add dictionary entry for " + property.getName(), e);
                    propertiesAdded=false;
                }
            }
        }
        if (propertiesAdded) {
            notifyNewPropertySubscribers();
        }
        return propertiesAdded;
    }

    //TODO: best way to do this? Upon deleting a location keep the reference intact, but remove from activatedEntity list
    public boolean removeActivatedEntity(Symbol refId) {
        if (this.activatedEntities.contains(refId)) {
            this.log.debug("[removeActivatedEntity] removed reference: " + refId);
            this.activatedEntities.remove(refId);
            return true;
        } else {
            this.log.warn("[removeActivatedEntity] reference not found: " + refId);
            return false;
        }
    }

    @Override
    public TemiV3PoseReference removeReference(Symbol refId) {
        activatedEntities.remove(refId);
        TemiV3PoseReference ref = getReference(refId);
        for (Term property : ref.properties) {
            propertiesHandled.removeIf(propHandled -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(property, propHandled));
//            if (propertiesHandled.stream().anyMatch(p -> PredicateHelper.predicatesMatch(p, property))) {
//            propertiesHandled.remove(property);
//            }
        }
        return super.removeReference(refId);
    }

    public void addActivatedEntity(Symbol refId) {
        if (!activatedEntities.contains(refId)) {
            this.activatedEntities.add(refId);
        }
    }

    public TemiV3PoseReference getLocationReferenceFromName(String name){
        for(TemiV3PoseReference ref : getAllReferences()){
            if(ref.getName().getName().equals(name)){
                return ref;
            }
        }
        return null;
    }

    public void addLocation(String name, Float x, Float y, Float theta, Float mapX, Float mapY, Float mapTheta) {

        List<Float> poseF = Arrays.asList(x,y,theta);
        List<Float> mapPoseF = Arrays.asList(mapX,mapY,mapTheta);

        //check if reference to this location already exists, if so just overwrite pose information
        TemiV3PoseReference existingRef = getLocationReferenceFromName(name);
        if (existingRef != null) {
            log.debug("[addLocation] updating existing locationRef");
            existingRef.setPose(poseF);
            existingRef.setMapPose(mapPoseF);
            if (!getActivatedEntities().containsKey(existingRef.refId)) {
                addActivatedEntity(existingRef.refId);
            }
        }else {
            //Make consultant aware of new location type
            List<Term> props = Arrays.asList(Factory.createPredicate(name, "X:" + getKBName()));
            addPropertiesHandled(props);

            //Get a new refId from consultant
            List<Variable> vars = new ArrayList<>();
            Variable var = Factory.createVariable("X:" + getKBName());
            vars.add(var);
            Map<Variable, Symbol> refIds = createReferences(vars);

            //Notify consultant we have an instance of new location type
            assertProperties(refIds.get(var), props);

            //add location name to created reference
            TemiV3PoseReference poseRef = getReference(refIds.get(var));
            poseRef.setName(Factory.createSymbol(name));
            poseRef.setPose(poseF);
            poseRef.setMapPose(mapPoseF);

        }
    }

}
