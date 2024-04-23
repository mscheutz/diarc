/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import edu.tufts.hrilab.llm.gson.PromptInfo;

import java.util.ArrayList;
import java.util.List;

public class TestUtterances {
    private String testName;
    public List<UtteranceData> utterances;

    /**
     * To keep track of what UtteranceData is currently being added to.
     */
    private int addIndex = -1;
    /**
     * To keep track of which tesUtteranceDatalet results to return next.
     */
    private int getterIndex = 0;


    public TestUtterances(String testName) {
        this.testName = testName;
        this.utterances = new ArrayList<>();
    }

    public void startNextUtterance() {
        utterances.add(new UtteranceData());
        addIndex = utterances.size() - 1;
    }
    public void addPromptInfo(PromptInfo promptInfo) {
        this.utterances.get(addIndex).setPromptInfo(promptInfo);
    }
    public void addUtteranceText(String utteranceText) {
        this.utterances.get(addIndex).setUtteranceText(utteranceText);
    }

    public void addDesiredSemantics(String desiredSemantics) {
        this.utterances.get(addIndex).setDesiredSemantics(desiredSemantics);
    }

}
