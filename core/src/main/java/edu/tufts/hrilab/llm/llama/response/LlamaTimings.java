/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama.response;

public class LlamaTimings {
    public float predicted_ms;
    public int predicted_n;
    public float predicted_per_second;
    public float predicted_per_token_ms;
    public float prompt_ms;
    public int prompt_n;
    public float prompt_per_second;
    public float prompt_per_token_ms;
}
