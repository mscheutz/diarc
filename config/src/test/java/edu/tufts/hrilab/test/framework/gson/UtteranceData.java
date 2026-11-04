/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import edu.tufts.hrilab.llm.gson.PromptInfo;

public class UtteranceData {

    public String utteranceText;
    public String desiredSemantics;
    public PromptInfo promptInfo;


    public void setPromptInfo(PromptInfo promptInfo) {
        this.promptInfo = promptInfo;
    }

    public void setUtteranceText(String utteranceText) {
        this.utteranceText = utteranceText;
    }

    public void setDesiredSemantics(String desiredSemantics) {
        this.desiredSemantics = desiredSemantics;
    }
}
