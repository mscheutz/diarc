package edu.tufts.hrilab.llm.hf.request;

import java.util.HashMap;
import java.util.Map;

public class T5BaseCompletionRequestBody {
    public String inputs;
    public Map<String,Boolean> options;

    private void init() {
        options = new HashMap<>();
        options.put("wait_for_model", false);
        options.put("use_cache", false);
    }

    public T5BaseCompletionRequestBody (String p) {
        init();
        inputs = p;
    }

    public void setWait_for_model(boolean wait_for_model) {
        options.put("wait_for_model", wait_for_model);
    }

    public void setUse_cache(boolean use_cache) {
        options.put("use_cache", use_cache);
    }
}
