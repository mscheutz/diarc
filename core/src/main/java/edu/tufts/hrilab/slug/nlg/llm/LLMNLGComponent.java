/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg.llm;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.slug.common.Utterance;

public class LLMNLGComponent extends DiarcComponent implements NLGInterface {

    public LLMNLGComponent() {
        super();
    }

    @Override
    public Utterance convertSemanticsToText(Utterance input) {
        String inputString = input.getWordsAsString();
        Utterance output;

        log.info("parseUtterance: " + inputString);

        try {
            output = TRADE.getAvailableService(new TRADEServiceConstraints().name("convertSemanticsToTextLLMService").argTypes(String.class)).call(Utterance.class, inputString);
        } catch (TRADEException ex) {
            log.error("Error calling convertSemanticsToTextLLMService.", ex);
            output = input;
        }

        return output;
    }

}
