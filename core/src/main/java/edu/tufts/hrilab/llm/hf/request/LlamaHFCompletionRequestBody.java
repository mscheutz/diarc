package edu.tufts.hrilab.llm.hf.request;

import java.util.HashMap;
import java.util.Map;

import edu.tufts.hrilab.llm.LlamaHFChat;

public class LlamaHFCompletionRequestBody {
    public String inputs;
    public Map<String,Object> parameters;
    public Map<String,Boolean> options;

    private void init() {
        parameters = new HashMap<>();
        parameters.put("temperature", 0.6f);
        parameters.put("top_p", 0.9f);
        parameters.put("top_k", 50);
        parameters.put("repetition_penalty", 1.0f);
        parameters.put("max_new_tokens", 100);
        parameters.put("return_full_text", false);
        parameters.put("max_time", 5);

        options = new HashMap<>();
        options.put("wait_for_model", false);
        options.put("use_cache", false);
    }

    public LlamaHFCompletionRequestBody () {
        init();
    }

    public LlamaHFCompletionRequestBody (String p) {
        init();
        inputs = p;
    }

    public LlamaHFCompletionRequestBody (LlamaHFChat chat) {
        init();
        inputs = chat.toPromptString();
    }

    public void setTemperature(float temperature) {
        parameters.put("temperature", temperature);
    }

    public void setTop_k(int top_k) {
        parameters.put("top_k", top_k);
    }

    public void setTop_p(float top_p) {
        parameters.put("top_p", top_p);
    }

    public void setRepetition_penalty(float repetition_penalty) {
        parameters.put("repetition_penalty ",repetition_penalty);
    }

    public void setReturn_full_text(boolean return_full_text) {
        parameters.put("return_full_text", return_full_text);
    }

    public void setNum_return_sequences(int num_return_sequences) {
        parameters.put("num_return_sequences", num_return_sequences);
    }

    public void setWait_for_model(boolean wait_for_model) {
        options.put("wait_for_model", wait_for_model);
    }

    public void setUse_cache(boolean use_cache) {
        options.put("use_cache", use_cache);
    }

    public void setMax_time(float max_time) {
        parameters.put("max_time", max_time);
    }

    public void setMax_new_tokens(int max_new_tokens) {
        parameters.put("max_new_tokens", max_new_tokens);
    }
}

