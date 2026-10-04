/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.llm.openai.OpenaiClient;
import edu.tufts.hrilab.slug.common.Utterance;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

public class SimpleNLGComponent extends DiarcComponent {
    final protected Logger log = LoggerFactory.getLogger(this.getClass());

    private NLG nlg = null;
    public String lastTranslation;
    private String modifierService = null;

    public SimpleNLGComponent() {
        super();
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("modifierService").hasArg().argName("string").desc("TRADE service used to modify outgoing utterances").build());
        return options;
    }

    @Override
    protected void parseArgs (CommandLine cmdLine) {
        if (cmdLine.hasOption("modifierService")) {
            modifierService = cmdLine.getOptionValue("modifierService");
            log.debug("Component will use TRADEService \"" + modifierService + "\" to modify utterances");
        }
    }

    @Override
    protected void init () {
        // create a local NLG component
        nlg = new NLG(modifierService);
    }

    @TRADEService
    @Action
    public Utterance convertSemanticsToText(Utterance u) {
        log.debug("utterance to translate: "+u);
        //convertSemanticsToText(u);
        String realization = nlg.translate(u);
        log.debug("semantics for utterance: " + u.getSemantics());
        lastTranslation = realization;
        log.debug("generated text: " + realization);
        Utterance.Builder output = new Utterance.Builder(u);
        output.setWords(realization);
        return output.build();
    }
}

