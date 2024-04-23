/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class GSPChainPrompt extends DIARCPrompt {
    static private final Logger log = LoggerFactory.getLogger(GSPChainPrompt.class);
    public GSPChainPrompt(String name) {
        super(name);
    }

    @Override
    public String generate() {


//        Chat gspChat = new Chat();
        Completion completion= null;

        // ## (0) Speech simplifier
//        StringBuilder template_simplifier= new StringBuilder();
//        template_simplifier.append("Simplify this sentence (don't get rid of determiners and articles):\n sentence: \n");
//        template_simplifier.append(humanInput);
//        template_simplifier.append("\n simplified: ");
//
//
//        log.debug("(0) Speech simplifier");
//        log.debug(template_simplifier.toString());
//        try{
//            completion = (Completion) TRADE.callThe("chatCompletion", template_simplifier.toString());
//        } catch (TRADEException e) {
//            log.error("[generate]",e);
//        }
//
//        String simplifiedUtterance="";
//        if(completion != null) {
//            simplifiedUtterance=completion.getText();
//        }
//        log.info("simplifiedUtterance: "+simplifiedUtterance);

        String simplifiedUtterance= humanInput;
        //## (1) Speech Act Classification

        // TODO: these UtteranceTypes need to be updated to reflect the current DIARC types
        StringBuilder template_speech_act_classifier = new StringBuilder();
        template_speech_act_classifier.append("Decide whether the utterance below from a speaker to a listener is one of INSTRUCT, STATEMENT, GREETING, QUESTIONWH ('wh', questions), QUESTIONYN ('yes/no' questions), ACK (e.g. \" yes \" or \" ok \"), or UNKNOWN \n");
        template_speech_act_classifier.append("An INSTRUCT is an imperative statement or a request by the speaker to have the listener do an action or stop doing an action.\n");
        template_speech_act_classifier.append(" A QUESTIONWH is a 'wh' query(what, why, when, where, who) or request from a speaker for more information from the listener about the listeners knowledge, beliefs or perceptions \n" );
        template_speech_act_classifier.append("A QUESTIONYN is a 'yes/no' query or request from a speaker for more information from the listener about the listeners knowledge, beliefs or perceptions, but the speaker expects a yes or no for an answer \n");
        template_speech_act_classifier.append("A STATEMENT is a statement of fact or opinion that the speaker conveys to a listener.\n");
        template_speech_act_classifier.append("A GREETING is an expression of social connection establishing the start of the conversation.E.g., \"Hello\".\n");
        template_speech_act_classifier.append("A ACK is an acknowledgement(either \"yes\" or \"no\").\n");
        template_speech_act_classifier.append("A UNKNOWN is an utterance not one of the above.\n");

        template_speech_act_classifier.append("utterance: \n { \n");
        template_speech_act_classifier.append(simplifiedUtterance);
        template_speech_act_classifier.append("\n act:");

        log.debug("(1) Speech Act Classification");
        log.debug(template_speech_act_classifier.toString());

        try{
            completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, template_speech_act_classifier.toString());
        } catch (TRADEException e) {
            log.error("[generate]",e);
        }

        UtteranceType utteranceType=UtteranceType.UNKNOWN;
        if(completion != null) {
           utteranceType =UtteranceType.valueOf(completion.getText());
        }
        log.info("utteranceType: "+ utteranceType);

        String cpc;

        if(utteranceType.equals(UtteranceType.STATEMENT)){
            //## (2a) CPC associated with concepts because we have a STATEMENT Speech act.
            StringBuilder  template_cpc_concept_selector = new StringBuilder();
            template_cpc_concept_selector.append("Generate a single string (without spaces or other special characters) that represents the core propositional content of the utterance. Use the following procedure:\n");
            template_cpc_concept_selector.append("1. Select the core portion of the utterance that represents the property of an object or concept associated with the object that the speaker is telling the listener that the listener does not already know.\n");
            template_cpc_concept_selector.append("2. Then, compare the name and description of each of the properties below to the utterance. Narrow the list of properties to include only those with a semantically similar name or description to the core portion of the utterance.\n");
            template_cpc_concept_selector.append("3. If the narrowed list contains one property, then return its name. If it contains no properties, then generate a new symbol -- a short (4-5 character) string to represent this portion/property in the utterance. If it contains more than one property, then return AMBIGUOUS.\n");
            template_cpc_concept_selector.append("\n\n LIST OF AVAILABLE PROPERTIES/CONCEPTS \n:");
            //TODO:brad: I don't think this should be properties? maybe all facts in belief? all condition/effect states?
            //TODO:add in success effects from actions
            for(Symbol fact: facts){
                template_cpc_concept_selector.append(fact).append(" ");
            }
            template_cpc_concept_selector.append("utterance: \n");
            template_cpc_concept_selector.append(simplifiedUtterance);
            template_cpc_concept_selector.append("\n core proposition name:");

            log.debug("(2a) CPC associated with concepts because we have a STATEMENT Speech act.");
            log.debug(template_cpc_concept_selector.toString());

            try{
                completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, template_cpc_concept_selector.toString());
            } catch (TRADEException e) {
                log.error("[generate]",e);
            }

            if(completion != null) {
                cpc =completion.getText();
            } else {
                cpc = "";
            }
            log.info("cpc: "+cpc);

        } else if(utteranceType.equals(UtteranceType.INSTRUCT)) {

            StringBuilder template_cpc_action_selector = new StringBuilder();
            template_cpc_action_selector.append("Select an action from the list of available actions that is most relevant to the given utterance.\n");
            template_cpc_action_selector.append("To decide the applicable action, use the following procedure to systematically filter the most relevant action:\n");
            template_cpc_action_selector.append("1. Check the dialog_type. Narrow the list of actions to only include those actions that are ontic actions or world-modifying actions.\n");
            template_cpc_action_selector.append("2. Then, compare the name and description of the action to the utterance. Narrow the list of actions to include only those with a semantically similar name or description to the utterance.\n");
            template_cpc_action_selector.append("3. If the narrowed list contains one action, then return its name. If it contains no actions, then return NONE. If it contains more than one action, then return AMBIGUOUS.\n");
            template_cpc_action_selector.append("\nLIST OF AVAILABLE ACTIONS: \n");
            for(ActionDBEntry a: actions){
               // template_cpc_action_selector.append(a.getSignature(true)).append(" ");
                template_cpc_action_selector.append("{name: ").append(a.getType()).append(", ");
                template_cpc_action_selector.append("roles:[");
                for(ActionBinding role:a.getInputRoles()) {
                    template_cpc_action_selector.append(role.name).append(",");
                }
                template_cpc_action_selector.deleteCharAt(template_cpc_action_selector.length() - 1);
                template_cpc_action_selector.append("]");
                //template_cpc_action_selector.append("description: ").append(a.getDescription()).append("\n");
                template_cpc_action_selector.append("}\n");
            }
            template_cpc_action_selector.append("\nutterance: \n");
            template_cpc_action_selector.append(simplifiedUtterance);
            template_cpc_action_selector.append("\naction:");

            log.debug("(2b) CPC associated with Actions because we have an INSTRUCT action");
            log.debug(template_cpc_action_selector.toString());

            try{
                completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, template_cpc_action_selector.toString());
            } catch (TRADEException e) {
                log.error("[generate]",e);
            }

            if(completion != null) {
                cpc =completion.getText();
            } else {
                cpc = "";
            }
            log.info("cpc: "+cpc);
        } else {
            cpc = "";
        }

        //## (4) Identify a referrent object and its properties.
        //### Given an utterance, an action or a property (the core proposition), and its roles, return a set of properties

        StringBuilder template_referent_properties = new StringBuilder();
        template_referent_properties.append("Identify descriptors in the utterance that refer to an entity which is an argument or parameter in the core proposition of the utterance.");
        template_referent_properties.append("Use the following procedure:\n");
        template_referent_properties.append("1. For each of the roles, identify what descriptive terms refer to the parameter type from the utterance. \n");
        template_referent_properties.append("That is think of an entity is being referred to by the roles, and see what descriptors refer to this entity.\n");
        template_referent_properties.append("2. Build a predicates for each descriptive terms. The predicates have a functor name that is the descriptor and a variable name.\n");
        template_referent_properties.append("Hypothesize variable names (e.g., VAR0, VAR1, etc.). The predicates should be of the form `descriptive term(variable names)`\n");
        template_referent_properties.append("3. Compose the predicates into a list \n");
        template_referent_properties.append("4. Return the list as properties. \n\n");
        template_referent_properties.append("Remember, these are descriptive properties (adjectives in a sense) and do NOT include cues for adverbs, articles, determiners, pronouns etc.\n");
        template_referent_properties.append("Also remember that functors cannot contain spaces, so replace spaces with underscores.\n");
        template_referent_properties.append("Also, remember the output list of properties should not include the core proposition itself. \n");
        template_referent_properties.append("\nEXAMPLE\n");
        template_referent_properties.append("utterance: Pick up the blue ball\n");
        template_referent_properties.append("core proposition: pickup\n");
        template_referent_properties.append("roles: [\"?object\"]\n");
        template_referent_properties.append("properties: [\"blue(VAR0)\", \"ball(VAR0)\"]\n");
        template_referent_properties.append("\n");
        template_referent_properties.append("\nEXAMPLE\n");
        template_referent_properties.append("utterance: Stack the cup on any other cup\n");
        template_referent_properties.append("core proposition: stack\n");
        template_referent_properties.append("roles: [\"?object1\",\"?object2\"]\n");
        template_referent_properties.append("properties: [\"cup(VAR0)\", \"cup(VAR1)\"]\n");
        template_referent_properties.append("\n");

        template_referent_properties.append("list of possible properties:\n");
        for(Term p: properties){
            template_referent_properties.append(p.toString()).append(", ");
        }
        template_referent_properties.deleteCharAt(template_referent_properties.length() - 1).deleteCharAt(template_referent_properties.length() - 1);
        template_referent_properties.append("\n\n");
        template_referent_properties.append("utterance: \n");
        template_referent_properties.append(simplifiedUtterance).append("\n");
        template_referent_properties.append("core proposition: \n");
        template_referent_properties.append(cpc).append("\n");
//        template_referent_properties.append("roles: \n");
//        //TODO:brad:there will be issues with overloaded signatures
//        //TODO:brad: some tume the cpc is the whole signature
//        List<ActionDBEntry> cpcActions =actions.stream().filter(a -> a.getType().equals(cpc)).collect(Collectors.toList());
//        if(cpcActions.size() ==0) {
//            log.error("[generate] no action found for cpc: "+cpc);
//            return "cpc error";
//        } else if (cpcActions.size()> 1) {
//            log.warn("multiple cpc actions found, picking the first... : "+cpcActions);
//        }
//        for(ActionBinding b: cpcActions.get(0).getInputRoles()){
//            template_referent_properties.append(b.name).append(",");
//        }
//        template_referent_properties.deleteCharAt(template_referent_properties.length()-1);
        template_referent_properties.append("\nproperties:\n");



        log.debug("(4) Identify a referent object and its properties. ");
        log.debug(template_referent_properties.toString());

        try{
            completion = (Completion) TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, template_referent_properties.toString());
        } catch (TRADEException e) {
            log.error("[generate]",e);
        }

        String chain_referent_properties ="";
        if(completion != null) {
            chain_referent_properties =completion.getText();
        }
        log.info("chain_referent_properties: "+chain_referent_properties);


        //## (5) Ref Res: For each variable determine its cognitive status
        //### For each variable we are figuring out the one or more givenness hierarchy statuses

        StringBuilder template_cognitive_status= new StringBuilder();
        template_cognitive_status.append("For each variable in the properties, decide which ONE (and only one) of the following five cognitive statuses the variables in the below properties could fall into:\n");
        template_cognitive_status.append("statuses: [INFOCUS, ACTIVATED, FAMILIAR, DEFINITE, INDEFINITE]\n");

        template_cognitive_status.append("As shown in the table below, the Givenness Hierarchy is comprised of six hierarchically nested tiers of cognitive status, where information with one cognitive status can be inferred to also have all lower statuses. Each level of the GH is \"cued\" by a set of linguistic forms, as seen in the table. For example, the second row of the table shows that the definite use of \"this\" can be used to infer that the speaker assumes the referent to be at least activated to their interlocutor.\n");
        template_cognitive_status.append("\n");
        template_cognitive_status.append("Cognitive Status | Mnemonic Status | Form |\n");
        template_cognitive_status.append("-----------------|-----------------|------|\n");
        template_cognitive_status.append("INFOCUS | in the focus of attention | it |\n");
        template_cognitive_status.append("ACTIVATED | in short term memory | this,that,this N |\n");
        template_cognitive_status.append("FAMILIAR | in long term memory| that N |\n");
        template_cognitive_status.append("DEFINITE | in long term memory  or new | the N |\n");
        template_cognitive_status.append("INDEFINITE | new or hypothetical | a N |\n");
        template_cognitive_status.append("\n");
        template_cognitive_status.append("When deciding the one cognitive status for each variable, use the table above and compare the form (pronoun, determiner, article) of the utterance to its status.\n");
        template_cognitive_status.append("\nExample:\n");
        template_cognitive_status.append("utterance: Pick up the blue ball\n");
        template_cognitive_status.append("properties: [\"blue(VAR0)\", \"ball(VAR0)\"]\n");
        template_cognitive_status.append("cognitive status: [\"DEFINITE(VAR0)\"]\n");
        template_cognitive_status.append("\n");
        template_cognitive_status.append("utterance: ").append(simplifiedUtterance).append("\n");
        template_cognitive_status.append("properties: ").append(chain_referent_properties).append("\n");
        template_cognitive_status.append("cognitive status:");

        log.debug("(5) Ref Res: For each variable determine it's cognitive status.");
        log.debug(template_cognitive_status.toString());

        try{
            completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, template_cognitive_status.toString());
        } catch (TRADEException e) {
            log.error("[generate]",e);
            return "cognitive status error";
        }
        String chain_cognitive_status = "";
        if(completion != null) {
            chain_cognitive_status =completion.getText();
        }
        log.info("chain_cognitive_status: "+chain_cognitive_status);

        //TODO:brad: should this return a string or utterance?
        StringBuilder resultingSemantics = new StringBuilder();
        resultingSemantics.append(utteranceType).append(" ").append(cpc).append(" ").append(chain_referent_properties).append(" ").append(chain_cognitive_status);
        log.info("resulting semantics: "+resultingSemantics);
        return resultingSemantics.toString();
    }
}
