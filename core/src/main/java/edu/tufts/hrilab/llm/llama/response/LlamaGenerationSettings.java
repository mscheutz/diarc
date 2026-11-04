/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama.response;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LlamaGenerationSettings {
    public float frequency_penalty;
    public String grammar;
    public boolean ignore_eos;
    public float[][] logit_bias;
    public float mirostat;
    public float mirostat_eta;
    public float mirostat_tau;
    public String model;
    public int n_ctx;
    public int n_keep;
    public int n_predict;
    public int n_probs;
    public boolean penalize_nl;
    public float presence_penalty;
    public int repeat_last_n;
    public float repeat_penalty;
    public long seed;
    public String[] stop;
    public boolean stream;
    public float temp;
    public float tfs_z;
    public int top_k;
    public float top_p;
    public float typical_p;
}
