/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.common.Utterance;

import edu.tufts.hrilab.interfaces.NLGInterface;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SimpleNLGComponent extends DiarcComponent implements NLGInterface {
    final protected Logger log = LoggerFactory.getLogger(this.getClass());

    public String lastTranslation;

    // create a local NLG component
    NLG nlg = new NLG();
    
    public SimpleNLGComponent() {
        super();
    }

    @Override
    public Utterance convertSemanticsToText(Utterance u) {
        log.debug("utterance to translate: "+u);
        //convertSemanticsToText(u);
        String realization = nlg.translate(u);
        log.debug("semantics for utterance: " + u.getSemantics());
        lastTranslation = realization;
        log.debug("generated text: " + realization);
        u.setWords(realization);
        return u;
    }
}

