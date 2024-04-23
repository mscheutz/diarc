/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.ActionDatabase;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.consultant.util.Utilities;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public abstract class  DIARCPrompt {

    private static final Logger log = LoggerFactory.getLogger(DIARCPrompt.class);
    protected String name;
    protected String humanInput="";
    protected List<Term> facts = new ArrayList<>();
    protected Set<ActionDBEntry> actions = new HashSet<>();
    //TODO:Brad: Observers?
    protected List<Term> properties = new ArrayList<>();
    protected Map<Symbol,List<Term>> entities= new HashMap<>();

    protected long latestTimeStamp= -1l;

    //todo: Add save to file
    public DIARCPrompt(String name) {
        this.name = name;
    }

    /**
     * Generates the text of the prompt based on the information collected from DIARC
     * @return String that will be sent to the LLM
     */
    public abstract String generate();


    /**
     * Add the raw NL from the human to the utterance/
     * @param input nl representation of input from the human
     */
    public void setHumanInput(String input){
        humanInput=input;
    }

    /**
     * Updates prompt with relevant state information from DIARC. This info will be used within generate() to gerneate the prompt.
     */

    public void getStateFromDIARC(){

        DIARCState state = new DIARCState();

        try {
            state.addFacts(TRADE.getAvailableService(new TRADEServiceConstraints().name("getFacts")).call(List.class));
        } catch (TRADEException e) {
            log.error("[getStateFromDIARC]",e);
        }

        state.addActions(Database.getInstance().getActionDB().getAllActions());

       TRADE.getAvailableServices(new TRADEServiceConstraints().name("getPropertiesHandled")).forEach(x -> {
           try {
               state.addProperties(x.call(List.class));
           } catch (TRADEException e) {
               log.error("Exception getting properties from consultants.", e);
           }
               }
       );

        //Iterates through all consultants (vision, location, etc) to generate observable objects
        Collection<TRADEServiceInfo> consultantServices = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getInitialDomain"));
        for (TRADEServiceInfo consultantService : consultantServices) {
            try {
                List<Symbol> refIds = consultantService.call(List.class,new ArrayList<>());
                //String kbname = (String) TRADE.callThe(consultant, "getKBName");

                Map<Symbol,List<Term>> localEntities= new HashMap<>();
                for (Symbol refId : refIds) {
                    List<Term> properties =  TRADE.getAvailableService(new TRADEServiceConstraints().inGroups(consultantService.getGroups().toArray(new String[0])).name("getAssertedProperties").argTypes(Symbol.class)).call(List.class, refId);
                    localEntities.put(refId,properties);
                }

                state.addEntities(localEntities);
            } catch (Exception e) {
                log.error("Exception getting info from consultant: "+consultantService.toString(), e);
            }
        }
        state.addTimeStamp();

        updatePromptState(state);
    }

    /**
     * Updates prompt state info with most recent state info from DIARC. The default is to overwrite everything, if you want a different behavior override this
     * @param stateUpdate sate information from DIARC
     */
    protected void updatePromptState(DIARCState stateUpdate){
        this.facts= stateUpdate.facts;
        this.actions=stateUpdate.actions;
        this.properties =stateUpdate.properties;
        this.entities=stateUpdate.entities;
    }

    public long getLatestTimeStamp(){
        if(latestTimeStamp == -1l){
            log.warn("[getLatestTimeStamp] called with default value. Prompt hasn't ever gotten a state update");
        }
        return latestTimeStamp;
    }

    /**
     * Encapsulating class for the state information present in DIARC at a current time.
     */
    protected class DIARCState {

        private long timeStamp=-1L;

        private final List<Term> facts = new ArrayList<>();
        private final Set<ActionDBEntry> actions = new HashSet<>();

        private final List<Term> properties = new ArrayList<>();
        private final Map<Symbol,List<Term>> entities = new HashMap<>();


        //TODO:brad:determine how/when we want to update state across prompt generations.
        public void addFacts(List<Term> facts){
            this.facts.addAll(facts);
        }

        public void addActions(Set<ActionDBEntry> actions){
            this.actions.addAll(actions);
        }

        public void addProperties(List<Term> properties){
            this.properties.addAll(properties);
        }
        public void addEntities(Map<Symbol,List<Term>> entities){
            this.entities.putAll(entities);
        }

        public void addTimeStamp(){
            timeStamp = System.currentTimeMillis();
        }

        public long getTimeStamp(){
            return timeStamp;
        }
    }
}
