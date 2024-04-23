/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama.request;

import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Chat;

public class LlamaCompletionRequestBody {
    public String prompt;
    public float temperature = 0.8f;
    public boolean stream = false;
    public int top_k = 40;
    public float top_p = 0.9f;
    public float repeat_penalty = 1.1f;
    public String[] stop = {};

    public LlamaCompletionRequestBody () {

    }

    public LlamaCompletionRequestBody (String p) {
        prompt = p;
    }

    public LlamaCompletionRequestBody (Chat chat) {
        prompt = chat.toPromptString();
    }

    public void addStopWords (String[] stopWords) {
        stop = stopWords;
    }

    public void setTemperature(float temperatureFloat) {
        temperature = temperatureFloat;
    }
}