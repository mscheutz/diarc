/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama.response;

public class LlamaCompletionResponse {
    public String content;
    public LlamaGenerationSettings generation_settings;
    public String model;
    public String prompt;
    public boolean stop;
    public boolean stopped_eos;
    public boolean stopped_limit;
    public String stopping_word;
    public LlamaTimings timings;
    public int tokens_cached;
    public int tokens_evaluated;
    public int tokens_predicted;
    public boolean truncated;
}