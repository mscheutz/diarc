/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class NaiveDIARCPrompt extends DIARCPrompt {
    public NaiveDIARCPrompt(String name) {
        super(name);
    }

    @Override
    public String generate() {
        StringBuilder promptText= new StringBuilder();

        promptText.append("create a formal representation of the semantics of this utterance: "+humanInput+" . Using the information below \n");

        promptText.append("here are all of the facts:\n");
        for(Term f: facts){
            promptText.append(f).append(" ");
        }

        promptText.append("\n here are all of the actions:\n");
        for(ActionDBEntry a :actions){
            promptText.append(a.getSignature(true)).append(" ");
            //This is more or less what Matt was doing
            promptText.append("this is how you would tell someone to do it ").append(a.getDescription()).append(" ");
        }

//        promptText.append("\n here are all of the possible states: \n");
//
//        //TODO:brad: these could probably be consolidated
//        Set<Condition> conditions = new HashSet<>();
//        actions.forEach(x -> conditions.addAll(x.getConditions()));
//        for(Condition c :conditions){
//            promptText.append(c).append(" ");
//        }
//        Set<Effect> effects = new HashSet<>();
//        actions.forEach(x -> effects.addAll(x.getEffects()));
//        for(Effect e :effects){
//            promptText.append(e).append(" ");
//        }

        promptText.append("\n here are all of the properties an entity could have:\n");
        for(Term prop: properties){
            promptText.append(prop).append(" ");
        }

        promptText.append("\n here are all of the entities you know about:\n");
        for(Map.Entry<Symbol, List<Term>> e: entities.entrySet()){
            promptText.append(e.getKey()).append(" it has the properties: ").append(e.getValue()).append(" ");
        }

        return promptText.toString();
    }

    //TODO:brad: dropping this here because I deleted Mar's other class, we would likely want to have separate DIARCPrompts extension for NLU and NLG, but I didn't have time to add that
//    @Override String generateNaiveNLG(){
//        if(u.semantics.getName().equals("ack")) {
////            String realization = nlg.translate(u);
////            u.words = new ArrayList<>(Arrays.asList(realization.trim().split(" ")));
//        } else {
//            String prompt = "Your name is " + u.getSpeaker() +". Make sure you use \"I\" instead of your name where " +
//                    "appropriate. ";
//            prompt += "You are addressing " + u.getListeners().toString() + ". Make sure to use \"you\" instead of " +
//                    "their names where appropriate. ";
//            prompt += "Create a very concise one sentence response given the following predicates: ";
//            prompt += u.semantics.toString();
//            prompt += u.bindings.toString();
//            prompt += u.supplementalSemantics.toString();
//            prompt += u.getIndirectSemantics().toString();
//            Completion realization;
//            try {
//                realization = (Completion) TRADE.callThe("chatCompletion", prompt);
//            } catch (TRADEException e) {
//                throw new RuntimeException(e);
//            }
//            u.words = new ArrayList<>(Arrays.asList(realization.text.trim().split(" ")));
//        }
//
//    }

}
