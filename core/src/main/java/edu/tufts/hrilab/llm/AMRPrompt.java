/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AMRPrompt extends DIARCPrompt {

    static private final Logger log = LoggerFactory.getLogger(AMRPrompt.class);
    public AMRPrompt(String name) {
        super(name);
    }
    @Override
    public String generate() {

        StringBuilder promptString= new StringBuilder();
        promptString.append("generate an AMR for the utterance: \"");
        promptString.append(humanInput);
        promptString.append("\".");


        log.debug("AMR Prompt:");
        log.debug(promptString.toString());

        String result="";
        try{
           Completion completion = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, promptString.toString());

            String simplifiedUtterance="";
            if(completion != null) {
                result=completion.getText();
            }
        } catch (TRADEException e) {
            log.error("[generate]",e);
        }

        log.info("generated AMR:\n"+result);

        return result;
    }
}
