/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg.llm;

import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.slug.common.Utterance;

public class LLMNLGComponent extends DiarcComponent {

    public LLMNLGComponent() {
        super();
    }

    @TRADEService
    @Action
    public Utterance convertSemanticsToText(Utterance input) {
        Utterance output;

        log.info("convertSemanticsToText: " + input.getWordsAsString());

        try {
            output = TRADE.getAvailableService(new TRADEServiceConstraints().name("convertSemanticsToTextLLMService").argTypes(Utterance.class)).call(Utterance.class, input);
        } catch (TRADEException ex) {
            log.error("Error calling convertSemanticsToTextLLMService.", ex);
            output = input;
        }

        return output;
    }

}
