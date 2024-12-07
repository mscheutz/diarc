/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.config.project;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


// the Project config
public class ProjectConfig extends DiarcConfiguration {
    // for logging
    protected static Logger log = LoggerFactory.getLogger(ProjectConfig.class);

    // start the configuration
    @Override
    public void runConfiguration() {
        // use my name as speaker and give it the name of the shopping robot
        // and supply a custom set of instructions for the buttons
//        createInstance(edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent.class,
//                "-config project/speechinput.simspeech -speaker hengxu -addressee roboshopper");

        // create the sim output
        createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class);

        // make the connection to the CS LLM
//        createInstance(edu.tufts.hrilab.llm.LLMComponent.class, "-endpoint http://vm-llama.eecs.tufts.edu:8080");

        // make it call the parsing and LLM function we're supposed to write
//        createInstance(edu.tufts.hrilab.slug.parsing.llm.Hw2LLMParserComponent.class, "-service parseIt");
        //createInstance(edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

        createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");

        createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class);

        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

        createInstance(edu.tufts.hrilab.slug.nlg.SimpleNLGComponent.class);

        // use the supermarket domain with defined agents for Prolog, and use the basic dialogue scripts plus
        // the supermarket actions for action interpreter
        // removed domains/supermarket.pl domains/supermarketRefactor.asl
        String gmArgs = "-beliefinitfile demos.pl agents/projectagents.pl " +
                "-asl core.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl " +
                "-goal listen(self)";
        createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class, gmArgs);

        // give it the name of the shopping robot
//        createInstance(edu.tufts.hrilab.supermarket.SupermarketComponent.class, "-agentName roboshopper");

        createInstance(edu.tufts.hrilab.vla.VLAComponent.class);
    }
}
