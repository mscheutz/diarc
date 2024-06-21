/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ContextConsultant extends Consultant<ContextReference> {

    RootContext root;
    //TODO:brad: do we need to care about race conditions?
    List<ContextReference> activatedEntities;
    Map<Long, ContextReference> contextCache;

    public ContextConsultant(RootContext root, String kbName) {
        super(ContextReference.class, kbName);
        this.root = root;
        activatedEntities = new ArrayList<>();
        contextCache = new HashMap<>();
    }

    @Override
    protected <U> U localConvertToType(Symbol refId, Class<U> type) {
        ContextReference ref = getReference(refId);
        if(ref == null){
            log.warn("[localConvertToType] no refrence found for id: "+refId+" returning null");
            return null;
        }
        if (type.isAssignableFrom(Context.class)) {
            // Point3d
            return type.cast(ref.context);
        } else {
            log.warn("[localConvertToType] un assignable type: "+type);
            return null;
        }
    }

    @Override
    protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
        ContextReference ref = getReference(refId);
        Context context = ref.context;
        if (type.isAssignableFrom(Context.class)) {
            // Point3d
            return type.cast(context);
        } else {
            return null;
        }
    }

    @Override
    public Map<Symbol, Double> getActivatedEntities() {
        this.log.debug("[getActivatedEntities]");
        Map<Symbol, Double> vals = new HashMap<>();

        for (ContextReference activatedEntity : this.activatedEntities) {
            Symbol ref = activatedEntity.refId;
            vals.put(ref, 10.0D);
        }

        return vals;
    }

//    protected ContextReference addContext(long id, Context c) {
//        String idString = Long.toString(id);
//        List<Term> props = new ArrayList<>();
//        props.add(Factory.createPredicate(idString, "X:context"));
//        addPropertiesHandled(props);
//
//        //Get a new refId from consultant
//        List<Variable> vars = new ArrayList<>();
//        Variable var = Factory.createVariable("X:context");
//        vars.add(var);
//        Map<Variable, Symbol> refIds = createReferences(vars);
//
//        //Notify consultant we have an instance of new location type
//        assertProperties(refIds.get(var), Utilities.convertToProperties(props));
//
//        //add location name to created reference
//        ContextReference contextReference = getReference(refIds.get(var));
//        contextReference.setContext(c);
//
//        //TODo:brad: generalize this/ move it somewhere else;
//        //Notify RR about salience of learned ref
//        try {
//            TRADE.callThe("setLastLearnedRef", contextReference.refId);
//        } catch (TRADEException e) {
//            log.error("exception updating RR with pose ref salience", e);
//        }
//
//        //inform parser of new location
//        log.debug("adding pose " + idString + " to parser dict");
//        //TODO reimplement homophone permutations
//        try {
//            TRADE.callThe("injectDictionaryEntry", idString, "RN", idString, "VAR");
//            TRADE.callThe("injectDictionaryEntry", idString, "REF", idString, "DEFINITE");
//        } catch (TRADEException e) {
//            log.error("unable to add dictionary entry for " + idString, e);
//        }
//        return contextReference;
//    }


    private ContextReference createAndAddReference(Context c){
        ContextReference ref = new ContextReference(getNextReferenceId(),c);
        addReference(ref);
        return ref;
    }

    @TRADEService
    public void makeContextSalient(long id) {
        activatedEntities = new ArrayList<>();
        //if it's cached you don't need to search for it
        if (contextCache.containsKey(id)) {
            activatedEntities.add(contextCache.get(id));
        } else {
            //search top level children breadth first
            for (Context c : root.getChildContexts().getChildrenContexts()) {
                if (c.getId() == id) {
                    activatedEntities.add(createAndAddReference(c));
                    return;
                }
            }
            //otherwise DFS
            for (Context c : root.getChildContexts().getChildrenContexts()) {
                Context result = searchForID(id, c);
                if (result != null) {
                    activatedEntities.add(createAndAddReference(result));
                    return;
                }
            }
        }
    }

    private Context searchForID(long id, Context c) {
        if (c.getId() == id) {
            return c;
        } else if (!c.getChildContexts().isEmpty()) {
            for (Context child : c.getChildContexts().getChildrenContexts()) {
                Context result = searchForID(id, child);
                if (result != null) {
                    return result;
                }
            }
        }
        return null;
    }

    @Action
    @TRADEService
    public Context getContextFromId(long id) {
        for (Context c : root.getChildContexts().getChildrenContexts()) {
            return searchForID(id, c);
        }
        return null;
    }

    @Action
    @TRADEService
    public Justification getSuccessFromRef(Symbol refId) {
        return getReference(refId).getContext().getJustification();
//        ActionContextDescription d = (ActionContextDescription) c.getContextDescription();
//        return d.getSuccessWithReason();
    }
    @Action
    @TRADEService
    public boolean getStatusFromRef(Symbol refId) {
       return getReference(refId).getContext().getStatus().isFailure();
//        ActionContextDescription d = (ActionContextDescription) c.getContextDescription();
//        return d.getSuccessWithReason();
    }
}
